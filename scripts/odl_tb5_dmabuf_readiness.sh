#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
#
# Two-host DMA-BUF raw-payload readiness gate.
#
# Run this on one OdinLink host, as root, after the same build has been
# staged on both hosts.  It deliberately does not load/unload modules or
# reset counters: it measures deltas, leaving an already-working link alone.
#
# Example:
#   sudo bash scripts/odl_tb5_dmabuf_readiness.sh \
#     --peer root@192.168.2.83 --bin-dir /root/odl/build/tests
#
# It proves the driver path directly, without vLLM, Ray, or RCCL plugin
# selection: paired legacy + stream DMA-BUF transfers in both directions,
# followed by a DMA-BUF echo stress test.  A passing run requires the
# negotiated raw path and rejects framed fallback even if transfers succeed.

set -euo pipefail

PEER=
BIN_DIR=
REMOTE_BIN_DIR=
DEV=0
ITERS=25
WARMUP=5
SIZES="4K,64K,1M,4M"
EXTRA_ALLOC=hip
DEBUGFS_ROOT=/sys/kernel/debug/odl_tb5
TIMEOUT_SECS=180
KEEP_LOGS=0
RUN_ID="odl-dmabuf-readiness-${USER:-root}-$$"
LOCAL_LOG_DIR="${TMPDIR:-/tmp}/$RUN_ID"
REMOTE_LOG_DIR="/tmp/$RUN_ID"
SKIP_RC=3

usage() {
    cat <<'EOF'
Usage: sudo bash odl_tb5_dmabuf_readiness.sh --peer USER@HOST --bin-dir DIR [options]

Required:
  --peer USER@HOST       SSH destination for the other OdinLink host
  --bin-dir DIR          Directory containing the test binaries locally

Options:
  --remote-bin-dir DIR   Test directory on the peer (default: --bin-dir)
  --dev N                OdinLink device index (default: 0)
  --iters N              Counted echo iterations per size (default: 25)
  --warmup N             Echo warm-up iterations per size (default: 5)
  --sizes LIST           Comma-separated sizes, e.g. 4K,64K,1M,4M
  --extra-allocator NAME  Second echo round with this bench allocator:
                         amdgpu (VRAM DMA-BUF via libdrm_amdgpu), hip
                         (amdgpu + HIP import), or none (default: hip).
                         A clean SKIP on both hosts is recorded, not a fail.
  --timeout SEC          Per-test timeout (default: 180)
  --debugfs-root DIR     Debugfs OdinLink root (default: /sys/kernel/debug/odl_tb5)
  --keep-logs            Preserve local and peer logs after success
  -h, --help             Show this help

Prerequisites on BOTH hosts:
  * the same userspace build and odl_tb5 module are installed;
  * /dev/odl_tb5_N is present and the link is already connected;
  * protocol=0 and raw_payload=Y were used when loading the module;
  * no other program is using the OdinLink device during this run.

Pass criteria:
  * cur_raw_payload_ok=1 on both hosts before tests begin;
  * paired legacy and stream DMA-BUF integrity tests pass both directions;
  * DMA-BUF echo test passes all sizes with byte-for-byte integrity;
  * raw_tx_frames rises on both hosts; raw fallback/reject/mismatch and
    restart counters do not rise.
EOF
}

die() { printf 'FAIL: %s\n' "$*" >&2; exit 1; }
note() { printf '== %s\n' "$*"; }

while (($#)); do
    case "$1" in
        --peer) PEER=${2:?--peer requires a value}; shift 2 ;;
        --bin-dir) BIN_DIR=${2:?--bin-dir requires a value}; shift 2 ;;
        --remote-bin-dir) REMOTE_BIN_DIR=${2:?--remote-bin-dir requires a value}; shift 2 ;;
        --dev) DEV=${2:?--dev requires a value}; shift 2 ;;
        --iters) ITERS=${2:?--iters requires a value}; shift 2 ;;
        --warmup) WARMUP=${2:?--warmup requires a value}; shift 2 ;;
        --sizes) SIZES=${2:?--sizes requires a value}; shift 2 ;;
        --extra-allocator) EXTRA_ALLOC=${2:?--extra-allocator requires a value}; shift 2 ;;
        --timeout) TIMEOUT_SECS=${2:?--timeout requires a value}; shift 2 ;;
        --debugfs-root) DEBUGFS_ROOT=${2:?--debugfs-root requires a value}; shift 2 ;;
        --keep-logs) KEEP_LOGS=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) die "unknown option: $1" ;;
    esac
done

[[ -n "$PEER" ]] || die '--peer is required'
[[ -n "$BIN_DIR" ]] || die '--bin-dir is required'
[[ -n "$REMOTE_BIN_DIR" ]] || REMOTE_BIN_DIR=$BIN_DIR
[[ $DEV =~ ^[0-9]+$ ]] || die '--dev must be a non-negative integer'
[[ $ITERS =~ ^[1-9][0-9]*$ ]] || die '--iters must be positive'
[[ $WARMUP =~ ^[0-9]+$ ]] || die '--warmup must be non-negative'
[[ $TIMEOUT_SECS =~ ^[1-9][0-9]*$ ]] || die '--timeout must be positive'
[[ $EUID -eq 0 ]] || die 'run as root so debugfs counters are readable'
[[ -x "$BIN_DIR/odl_tb5_pair_dmabuf" ]] || die "missing $BIN_DIR/odl_tb5_pair_dmabuf"
[[ -x "$BIN_DIR/odl_tb5_bench_dmabuf" ]] || die "missing $BIN_DIR/odl_tb5_bench_dmabuf"

mkdir -p "$LOCAL_LOG_DIR"

# Quote one complete command for the remote shell. Commands below only contain
# paths/values supplied as script arguments and are passed as individual shell
# words before this function sees them.
peer_run() {
    ssh -o BatchMode=yes -o ConnectTimeout=10 "$PEER" "bash -lc $(printf '%q' "$*")"
}

stats_file="$DEBUGFS_ROOT/odl_tb5_$DEV/stats"
peer_stats_file="$DEBUGFS_ROOT/odl_tb5_$DEV/stats"

counter_local() {
    awk -v key="$1" '$1 == key { print $2; found=1 } END { exit !found }' "$stats_file"
}

counter_peer() {
    peer_run "awk -v key=$(printf '%q' "$1") '\$1 == key { print \$2; found=1 } END { exit !found }' $(printf '%q' "$peer_stats_file")"
}

require_ready_local() {
    [[ -r "$stats_file" ]] || die "stats unavailable: $stats_file"
    [[ -r /sys/module/odl_tb5/parameters/protocol ]] || die 'odl_tb5 is not loaded locally'
    [[ $(< /sys/module/odl_tb5/parameters/protocol) == 0 ]] || die 'local protocol is not OdinLink mode (protocol=0)'
    [[ $(< /sys/module/odl_tb5/parameters/raw_payload) =~ ^[Yy1]$ ]] || die 'local raw_payload is not enabled'
    [[ $(counter_local cur_raw_payload_ok) == 1 ]] || die 'local link did not negotiate raw payload'
}

require_ready_peer() {
    peer_run "test -r $(printf '%q' "$peer_stats_file") && test -r /sys/module/odl_tb5/parameters/protocol" || die 'peer stats or module unavailable'
    [[ $(peer_run 'cat /sys/module/odl_tb5/parameters/protocol') == 0 ]] || die 'peer protocol is not OdinLink mode (protocol=0)'
    [[ $(peer_run 'cat /sys/module/odl_tb5/parameters/raw_payload') =~ ^[Yy1]$ ]] || die 'peer raw_payload is not enabled'
    [[ $(counter_peer cur_raw_payload_ok) == 1 ]] || die 'peer link did not negotiate raw payload'
}

declare -A BEFORE_LOCAL BEFORE_PEER AFTER_LOCAL AFTER_PEER
COUNTERS=(raw_tx_frames raw_eligible_reject raw_unaligned_fallback raw_rx_len_mismatch tx_bytes_submitted)
ERROR_COUNTERS=(raw_eligible_reject raw_unaligned_fallback raw_rx_len_mismatch)

has_counter_local() {
    awk -v key="$1" '$1 == key { found=1 } END { exit !found }' "$stats_file"
}

has_counter_peer() {
    peer_run "awk -v key=$(printf '%q' "$1") '\$1 == key { found=1 } END { exit !found }' $(printf '%q' "$peer_stats_file")"
}

enable_optional_counters() {
    if has_counter_local restart_count && has_counter_peer restart_count; then
        COUNTERS+=(restart_count)
        ERROR_COUNTERS+=(restart_count)
    else
        note 'restart_count is not exported by both deployed modules; skipping that optional counter gate'
    fi
}

snapshot() {
    local phase=$1 key
    for key in "${COUNTERS[@]}"; do
        if [[ $phase == before ]]; then
            BEFORE_LOCAL[$key]=$(counter_local "$key")
            BEFORE_PEER[$key]=$(counter_peer "$key")
        else
            AFTER_LOCAL[$key]=$(counter_local "$key")
            AFTER_PEER[$key]=$(counter_peer "$key")
        fi
    done
}

start_peer() {
    local label=$1; shift
    local cmd="$*"
    # Close the SSH channel in the background subshell.  Without these
    # redirections the remote test inherits ssh's stdout and the launcher
    # blocks before it can start the matching sender.
    peer_run "mkdir -p $(printf '%q' "$REMOTE_LOG_DIR"); rm -f $(printf '%q' "$REMOTE_LOG_DIR/$label.rc"); (timeout $(printf '%q' "$TIMEOUT_SECS") $cmd >$(printf '%q' "$REMOTE_LOG_DIR/$label.log") 2>&1; echo \$? >$(printf '%q' "$REMOTE_LOG_DIR/$label.rc")) </dev/null >/dev/null 2>&1 & echo \$! >$(printf '%q' "$REMOTE_LOG_DIR/$label.pid")"
}

wait_peer() {
    local label=$1 elapsed=0 rc
    while ((elapsed < TIMEOUT_SECS + 15)); do
        if peer_run "test -f $(printf '%q' "$REMOTE_LOG_DIR/$label.rc")"; then
            rc=$(peer_run "cat $(printf '%q' "$REMOTE_LOG_DIR/$label.rc")")
            peer_run "cat $(printf '%q' "$REMOTE_LOG_DIR/$label.log")" | tee "$LOCAL_LOG_DIR/$label.peer.log"
            [[ $rc == 0 ]] || die "peer test $label exited $rc (log: $LOCAL_LOG_DIR/$label.peer.log)"
            return
        fi
        sleep 1
        ((++elapsed))
    done
    die "timed out waiting for peer test $label"
}

run_local() {
    local label=$1; shift
    note "$label"
    if ! timeout "$TIMEOUT_SECS" "$@" 2>&1 | tee "$LOCAL_LOG_DIR/$label.local.log"; then
        die "local test $label failed (log: $LOCAL_LOG_DIR/$label.local.log)"
    fi
}

run_local_background() {
    local label=$1; shift
    timeout "$TIMEOUT_SECS" "$@" >"$LOCAL_LOG_DIR/$label.local.log" 2>&1 &
    LOCAL_BG_PID=$!
}

wait_local_background() {
    local label=$1
    if ! wait "$LOCAL_BG_PID"; then
        cat "$LOCAL_LOG_DIR/$label.local.log" >&2 || true
        die "local test $label failed (log: $LOCAL_LOG_DIR/$label.local.log)"
    fi
    cat "$LOCAL_LOG_DIR/$label.local.log"
}

# Soft variants used by the extra-allocator round: record the exit code in
# a global instead of dying, so a clean SKIP (3) can be distinguished from
# a failure.  Both return 0 so set -e never fires on the child's rc.
LOCAL_RC=
run_local_rc() {
    local label=$1; shift
    if timeout "$TIMEOUT_SECS" "$@" >"$LOCAL_LOG_DIR/$label.local.log" 2>&1; then
        LOCAL_RC=0
    else
        LOCAL_RC=$?
    fi
    cat "$LOCAL_LOG_DIR/$label.local.log"
    return 0
}

# Soft variant used by the extra-allocator round: record PEER_RC instead of
# dying, so a clean SKIP (3) can be distinguished from a failure.
PEER_RC=
wait_peer_rc() {
    local label=$1 elapsed=0
    while ((elapsed < TIMEOUT_SECS + 15)); do
        if peer_run "test -f $(printf '%q' "$REMOTE_LOG_DIR/$label.rc")"; then
            PEER_RC=$(peer_run "cat $(printf '%q' "$REMOTE_LOG_DIR/$label.rc")")
            peer_run "cat $(printf '%q' "$REMOTE_LOG_DIR/$label.log")" | tee "$LOCAL_LOG_DIR/$label.peer.log"
            return 0
        fi
        sleep 1
        ((++elapsed))
    done
    die "timed out waiting for peer test $label"
}

# Second echo round with a GPU allocator.  A clean SKIP on BOTH hosts is
# reported and does not fail the gate (the DMA-heap round is the pass
# criterion); a real run must satisfy the same counter gate as the main
# round.  Asymmetric SKIP (one side ran, the other skipped) is a failure.
extra_allocator_round() {
    local alloc=$1
    local label="extra_${alloc}"
    note "DMA-BUF echo stress (extra allocator: $alloc): peer server, local client"
    snapshot before
    start_peer "$label" "$(printf '%q' "$REMOTE_BIN_DIR/odl_tb5_bench_dmabuf") server --dev $(printf '%q' "$DEV") --iters $(printf '%q' "$ITERS") --warmup $(printf '%q' "$WARMUP") --sizes $(printf '%q' "$SIZES") --allocator $(printf '%q' "$alloc")"
    sleep 1
    local local_rc peer_rc
    run_local_rc "$label" "$BIN_DIR/odl_tb5_bench_dmabuf" client --dev "$DEV" --iters "$ITERS" --warmup "$WARMUP" --sizes "$SIZES" --allocator "$alloc"
    local_rc=$LOCAL_RC
    wait_peer_rc "$label"
    peer_rc=$PEER_RC

    local local_skip=0 peer_skip=0
    grep -q 'SKIP:' "$LOCAL_LOG_DIR/$label.local.log" && local_skip=1
    grep -q 'SKIP:' "$LOCAL_LOG_DIR/$label.peer.log" && peer_skip=1

    if ((local_skip || peer_skip)); then
        ((local_skip == 1 && peer_skip == 1)) || die "extra allocator $alloc: asymmetric SKIP (one side ran, the other skipped)"
        note "extra allocator $alloc: SKIP on both hosts — recorded, not counted toward the gate"
        grep -m1 'SKIP:' "$LOCAL_LOG_DIR/$label.local.log"
        return 0
    fi
    ((local_rc == 0 && peer_rc == 0)) || die "extra allocator $alloc round failed (local rc=$local_rc peer rc=$peer_rc)"
    grep -q 'integrity OK' "$LOCAL_LOG_DIR/$label.local.log" || die "extra allocator $alloc did not report integrity OK"
    assert_counter_gate
    note "extra allocator $alloc: PASS — raw proof above"
}

assert_pair_receiver_real_dmabuf() {
    local peer_label=$1
    grep -q 'real=1' "$LOCAL_LOG_DIR/$peer_label.peer.log" || die "$peer_label did not obtain a real DMA-BUF on the peer"
}

assert_counter_gate() {
    local key local_delta peer_delta
    snapshot after
    for key in "${ERROR_COUNTERS[@]}"; do
        local_delta=$(( AFTER_LOCAL[$key] - BEFORE_LOCAL[$key] ))
        peer_delta=$(( AFTER_PEER[$key] - BEFORE_PEER[$key] ))
        (( local_delta == 0 )) || die "local $key increased by $local_delta"
        (( peer_delta == 0 )) || die "peer $key increased by $peer_delta"
    done
    local_delta=$(( AFTER_LOCAL[raw_tx_frames] - BEFORE_LOCAL[raw_tx_frames] ))
    peer_delta=$(( AFTER_PEER[raw_tx_frames] - BEFORE_PEER[raw_tx_frames] ))
    (( local_delta > 0 )) || die 'local raw_tx_frames did not increase: framed fallback or no payload'
    (( peer_delta > 0 )) || die 'peer raw_tx_frames did not increase: framed fallback or no payload'
    printf 'raw proof: local raw_tx_frames +%d, peer +%d; local bytes +%d, peer +%d\n' \
        "$local_delta" "$peer_delta" \
        "$(( AFTER_LOCAL[tx_bytes_submitted] - BEFORE_LOCAL[tx_bytes_submitted] ))" \
        "$(( AFTER_PEER[tx_bytes_submitted] - BEFORE_PEER[tx_bytes_submitted] ))"
}

cleanup() {
    local rc=$?
    if ((rc == 0 && KEEP_LOGS == 0)); then
        peer_run "rm -rf $(printf '%q' "$REMOTE_LOG_DIR")" >/dev/null 2>&1 || true
        rm -rf "$LOCAL_LOG_DIR"
    else
        printf 'logs retained in %s (peer: %s)\n' "$LOCAL_LOG_DIR" "$REMOTE_LOG_DIR" >&2
    fi
}
trap cleanup EXIT

note 'checking bilateral raw-payload readiness'
require_ready_local
require_ready_peer
enable_optional_counters
snapshot before

note 'paired legacy + stream DMA-BUF integrity: peer receives, local sends'
start_peer pair_peer_recv "$(printf '%q' "$REMOTE_BIN_DIR/odl_tb5_pair_dmabuf") recv -d $(printf '%q' "$DEV") 65536"
sleep 1
run_local pair_local_send "$BIN_DIR/odl_tb5_pair_dmabuf" send -d "$DEV" 65536
wait_peer pair_peer_recv
assert_pair_receiver_real_dmabuf pair_peer_recv

note 'paired legacy + stream DMA-BUF integrity: local receives, peer sends'
run_local_background pair_local_recv "$BIN_DIR/odl_tb5_pair_dmabuf" recv -d "$DEV" 65536
sleep 1
start_peer pair_peer_send "$(printf '%q' "$REMOTE_BIN_DIR/odl_tb5_pair_dmabuf") send -d $(printf '%q' "$DEV") 65536"
wait_peer pair_peer_send
wait_local_background pair_local_recv
grep -q 'real=1' "$LOCAL_LOG_DIR/pair_local_recv.local.log" || die 'local receiver did not obtain a real DMA-BUF'

note 'DMA-BUF echo stress: peer server, local client'
start_peer bench_server "$(printf '%q' "$REMOTE_BIN_DIR/odl_tb5_bench_dmabuf") server --dev $(printf '%q' "$DEV") --iters $(printf '%q' "$ITERS") --warmup $(printf '%q' "$WARMUP") --sizes $(printf '%q' "$SIZES")"
sleep 1
run_local bench_client "$BIN_DIR/odl_tb5_bench_dmabuf" client --dev "$DEV" --iters "$ITERS" --warmup "$WARMUP" --sizes "$SIZES"
wait_peer bench_server
grep -q 'integrity OK' "$LOCAL_LOG_DIR/bench_client.local.log" || die 'echo benchmark did not report integrity OK'

assert_counter_gate
case "$EXTRA_ALLOC" in
    none) note 'extra allocator round disabled (--extra-allocator none)' ;;
    amdgpu|hip) extra_allocator_round "$EXTRA_ALLOC" ;;
    *) die "invalid --extra-allocator: $EXTRA_ALLOC (expected amdgpu|hip|none)" ;;
esac
note 'PASS: direct raw DMA-BUF readiness gate passed (no vLLM/Ray/RCCL selection involved)'
