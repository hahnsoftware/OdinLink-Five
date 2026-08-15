#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
#
# Narrow raw-payload reproduction helper.
#
# Runs the DMA-buf echo bench one size at a time and snapshots the raw
# counters on BOTH hosts around each size, so a failing size can be
# correlated with raw_eligible_reject / raw_rx_len_mismatch deltas.
#
# Run on one OdinLink host as root, with the same build staged on both hosts
# and the link already up (module loaded, raw_payload negotiated).  It does
# not load/unload modules or reset counters — it measures deltas.
#
# Example:
#   sudo bash scripts/odl_tb5_dmabuf_repro.sh \
#     --peer root@192.168.2.83 --bin-dir /root/odl-t91zc/build/tests \
#     --sizes 4K,64K,1M,4M --iters 3 --warmup 1

set -euo pipefail

PEER=
BIN_DIR=
REMOTE_BIN_DIR=
DEV=0
ITERS=5
WARMUP=2
SIZES="4K,64K,1M,4M"
DEBUGFS_ROOT=/sys/kernel/debug/odl_tb5
TIMEOUT_SECS=120
RUN_ID="odl-dmabuf-repro-${USER:-root}-$$"
LOCAL_LOG_DIR="${TMPDIR:-/tmp}/$RUN_ID"
REMOTE_LOG_DIR="/tmp/$RUN_ID"

usage() {
    cat <<'EOF'
Usage: sudo bash odl_tb5_dmabuf_repro.sh --peer USER@HOST --bin-dir DIR [options]

Required:
  --peer USER@HOST       SSH destination for the other OdinLink host
  --bin-dir DIR          Directory containing the test binaries locally

Options:
  --remote-bin-dir DIR   Test directory on the peer (default: --bin-dir)
  --dev N                OdinLink device index (default: 0)
  --sizes LIST           Comma-separated sizes to correlate (default: 4K,64K,1M,4M)
  --iters N              Counted echo iterations per size (default: 5)
  --warmup N             Echo warm-up iterations per size (default: 2)
  --timeout SEC          Per-size timeout (default: 120)
  --debugfs-root DIR     Debugfs OdinLink root (default: /sys/kernel/debug/odl_tb5)
  -h, --help             Show this help

Read the local/peer raw counters before and after each size and print the
deltas.  A healthy raw path shows raw_tx_frames rising and
raw_eligible_reject / raw_unaligned_fallback / raw_rx_len_mismatch staying
at 0.  A size that only falls back to framed shows raw_tx_frames flat with
raw_eligible_reject rising on one or both hosts.
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
        --sizes) SIZES=${2:?--sizes requires a value}; shift 2 ;;
        --iters) ITERS=${2:?--iters requires a value}; shift 2 ;;
        --warmup) WARMUP=${2:?--warmup requires a value}; shift 2 ;;
        --timeout) TIMEOUT_SECS=${2:?--timeout requires a value}; shift 2 ;;
        --debugfs-root) DEBUGFS_ROOT=${2:?--debugfs-root requires a value}; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) die "unknown option: $1" ;;
    esac
done

[[ -n "$PEER" ]] || die '--peer is required'
[[ -n "$BIN_DIR" ]] || die '--bin-dir is required'
[[ -n "$REMOTE_BIN_DIR" ]] || REMOTE_BIN_DIR=$BIN_DIR
[[ $EUID -eq 0 ]] || die 'run as root so debugfs counters are readable'
[[ -x "$BIN_DIR/odl_tb5_bench_dmabuf" ]] || die "missing $BIN_DIR/odl_tb5_bench_dmabuf"

mkdir -p "$LOCAL_LOG_DIR"

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
    [[ $(counter_local cur_raw_payload_ok) == 1 ]] || die 'local link did not negotiate raw payload'
}

require_ready_peer() {
    peer_run "test -r $(printf '%q' "$peer_stats_file")" || die 'peer stats unavailable'
    [[ $(counter_peer cur_raw_payload_ok) == 1 ]] || die 'peer link did not negotiate raw payload'
}

COUNTERS=(raw_tx_frames raw_eligible_reject raw_unaligned_fallback raw_rx_len_mismatch tx_bytes_submitted)

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

declare -A BEFORE_LOCAL BEFORE_PEER AFTER_LOCAL AFTER_PEER

start_peer() {
    local label=$1; shift
    local cmd="$*"
    peer_run "mkdir -p $(printf '%q' "$REMOTE_LOG_DIR"); rm -f $(printf '%q' "$REMOTE_LOG_DIR/$label.rc"); (timeout $(printf '%q' "$TIMEOUT_SECS") $cmd >$(printf '%q' "$REMOTE_LOG_DIR/$label.log") 2>&1; echo \$? >$(printf '%q' "$REMOTE_LOG_DIR/$label.rc")) </dev/null >/dev/null 2>&1 & echo \$! >$(printf '%q' "$REMOTE_LOG_DIR/$label.pid")"
}

wait_peer() {
    local label=$1 elapsed=0 rc
    while ((elapsed < TIMEOUT_SECS + 15)); do
        if peer_run "test -f $(printf '%q' "$REMOTE_LOG_DIR/$label.rc")"; then
            rc=$(peer_run "cat $(printf '%q' "$REMOTE_LOG_DIR/$label.rc")")
            [[ $rc == 0 ]] || die "peer bench exited $rc (log: $REMOTE_LOG_DIR/$label.log)"
            return
        fi
        sleep 1
        ((++elapsed))
    done
    die "timed out waiting for peer bench $label"
}

run_one_size() {
    local size=$1 label="size_${size}"
    note "size $size: bench server on peer, client local"
    # Snapshot BEFORE the server starts: its first recv_dmabuf posts RX
    # staging immediately, which can already bump raw_eligible_reject.
    snapshot before
    start_peer "$label" "$(printf '%q' "$REMOTE_BIN_DIR/odl_tb5_bench_dmabuf") server --dev $(printf '%q' "$DEV") --iters $(printf '%q' "$ITERS") --warmup $(printf '%q' "$WARMUP") --sizes $(printf '%q' "$size")"
    sleep 1
    timeout "$TIMEOUT_SECS" "$BIN_DIR/odl_tb5_bench_dmabuf" client --dev "$DEV" --iters "$ITERS" --warmup "$WARMUP" --sizes "$size" 2>&1 | tee "$LOCAL_LOG_DIR/$label.local.log" || die "local bench failed for size $size"
    wait_peer "$label"
    snapshot after

    local d
    printf '  %-8s' "$size"
    for key in "${COUNTERS[@]}"; do
        d=$(( AFTER_LOCAL[$key] - BEFORE_LOCAL[$key] ))
        printf '  local:%s=%+d' "$key" "$d"
        d=$(( AFTER_PEER[$key] - BEFORE_PEER[$key] ))
        printf ' peer:%s=%+d' "$key" "$d"
    done
    printf '\n'
}

note 'checking bilateral raw-payload readiness'
require_ready_local
require_ready_peer

printf '  %-8s  %s\n' "size" "(deltas per size over one bench round per host)"
printf '  %-8s  %s\n' ""   "want: raw_tx_frames > 0, raw_eligible_reject = 0, raw_unaligned_fallback = 0, raw_rx_len_mismatch = 0"
for size in ${SIZES//,/ }; do
    run_one_size "$size"
done

note 'done (logs retained locally in '"$LOCAL_LOG_DIR"', peer '"$REMOTE_LOG_DIR"')'