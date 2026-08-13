#!/usr/bin/env bash
#
# odl-gate-dmabuf — Tier 1a rate gate over the zero-copy dmabuf path
# (the RCCL/GPU transport).  See tools/gates/README.md.
#
# What it protects:
#   * odl_tb5_submit_dmabuf / odl_tb5_recv_dmabuf (device-level sync DMA)
#   * the dmabuf ring separation and the reclaim path (drain-first, F2)
#   * the integrity of the fd-based zero-copy path RCCL/GPU depends on
#
# The gate is a *rate* gate, per lesson 2 of the README: it reports
# passes/N, never a bare verdict, because intermittent failures are the
# failure class this path actually produces.
#
# The trials sweep the ORIGINAL hang trigger in-process: just before the
# transfer loop the client probes EXPECT_REJECT once (an oversized submit
# the driver's up-front capacity check answers with -ENOSPC on a ring
# under pressure), and the very next operation is a normal in-ring
# transfer that would reuse the reclaimed ring.  The pre-F2 bug only
# fired on exactly that next-op-after-reject sequence; a single-size
# loop passed 16/16 while the sequence hung ~100 % of the time.
#
# Usage:
#   odl-gate-dmabuf.sh [--trials N] [--sizes a,b,c] [--iters N]
#                      [--warmup N] [--alt] [--timeout S]
#   --trials N    number of attempts (default $ODL_TRIALS, 20)
#   --sizes CSV   the per-trial in-ring transfer sizes (default 1048576)
#   --expect-reject CSV  sizes the client probes ONCE before the transfer
#                 loop to arm the capacity-reject trigger (default
#                 16777216 — the 16 MiB ring-full case)
#   --alt         alternate between the first and last transfer size
#                 on odd/even trials (legacy f2 hunt ALT mode)
#   --iters N     bench iterations per size (default 3)
#   --warmup N    bench warmups per size (default 1)
#   --timeout S   per-trial wall clock for the client (default 300)
#   --no-debugfs  run without the fork counter exports (leak check off)
#
# Env (also rig.conf): ODL_TRIALS, ODL_TRIAL_TIMEOUT, ODL_STOP_ON_FAIL,
# ODL_PEER, ODL_BUILD_DIR, ODL_BENCH, ODL_LIB...
#
# Exit: 0 all trials passed; 1 any failed; 2 skipped (missing prereq).

set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$DIR/od/common.sh"
. "$DIR/odl-link-check.sh"

# ---- defaults ----
TRIALS="${ODL_TRIALS:-20}"
# The trigger shape lives in EXPECT_REJECT: one oversized pro-ring-sized
# probe that the client ARMS the capacity check with (see bench
# --expect-reject), immediately followed by a normal in-ring transfer.
# The pre-F2 hang fired on exactly that next-op-after-reject sequence.
SIZES="1048576"
EXPECT_REJECT="16777216"
ITERS=3
WARMUP=1
ALT=0
TRIAL_TIMEOUT="${ODL_TRIAL_TIMEOUT:-300}"
NO_DEBUGFS="${ODL_NO_DEBUGFS:-0}"

while [ $# -gt 0 ]; do
    case "$1" in
        --trials)   TRIALS="$2"; shift 2 ;;
        --sizes)    SIZES="$2"; shift 2 ;;
        --expect-reject) EXPECT_REJECT="$2"; shift 2 ;;
        --iters)    ITERS="$2"; shift 2 ;;
        --warmup)   WARMUP="$2"; shift 2 ;;
        --alt)      ALT=1; shift ;;
        --timeout)  TRIAL_TIMEOUT="$2"; shift 2 ;;
        --no-debugfs) NO_DEBUGFS=1; shift ;;
        -h|--help)  sed -n '2,42p' "$0"; exit 0 ;;
        *) odl_die "unknown arg: $1" ;;
    esac
done

odl_setup "gate-dmabuf"
odl_link_check 2>/dev/null || odl_log "link-check not clean (continuing anyway — see odl-link-check)"

# binaries + debugfs prerequisites
[ -x "$ODL_BENCH" ] || odl_skip "bench binary missing: $ODL_BENCH"
if [ "$NO_DEBUGFS" = "0" ]; then
    odl_require_debugfs
fi

# ALT mode: alternate trials between first and last size token
ALT_A="${SIZES%%,*}"; ALT_B="${SIZES##*,}"
if [ "$ALT" = "1" ]; then
    odl_log "ALT mode: alternating ${ALT_A} / ${ALT_B}"
fi

pass=0; fail=0; hard=0

for t in $(seq 1 "$TRIALS"); do
    sz="$SIZES"
    if [ "$ALT" = "1" ]; then
        if [ $((t % 2)) -eq 1 ]; then sz="$ALT_A"; else sz="$ALT_B"; fi
    fi

    # observable state reset at both ends: counters + kernel log
    if [ "$NO_DEBUGFS" = "0" ]; then
        printf '1\n' >"$ODL_STATS_RESET" 2>/dev/null || true
        odl_peer "printf '1\n' > $ODL_STATS_RESET 2>/dev/null || true" 2>/dev/null || true
    fi
    odl_peer "dmesg -c >/dev/null 2>&1 || true" 2>/dev/null || true
    dmesg -c >/dev/null 2>&1 || true

    # fresh server per trial (the CLI server dies after each block; the
    # bench server here stays alive but a per-trial unit keeps the lease
    # short and the logs per-trial)
    unit="odl-dmabuf-$t-$$"
    systemd-run --unit="$unit" --collect --no-block \
        env LD_LIBRARY_PATH="$ODL_LD_LIBRARY_PATH" \
        "$ODL_BENCH" server --iters "$ITERS" --warmup "$WARMUP" \
            --sizes "$sz" --expect-reject "$EXPECT_REJECT" \
        >"$FARM/trial-$t-server.out" 2>&1 </dev/null || odl_die "server start failed"
    sleep 2

    # client driven from the peer under the wall-clock wrapper
    # shellcheck disable=SC2086
    ssh $ODL_SSH_OPTS "$ODL_PEER" \
        "env LD_LIBRARY_PATH=$ODL_LD_LIBRARY_PATH timeout $TRIAL_TIMEOUT \
         $ODL_BENCH client --iters $ITERS --warmup $WARMUP --sizes $sz \
            --expect-reject $EXPECT_REJECT" \
        >"$FARM/trial-$t-client.tmp" 2>&1
    rc=$?

    systemctl stop "$unit" 2>/dev/null || true
    systemctl reset-failed "$unit" 2>/dev/null || true

    # kernel-visible failure: the 5 s submit-timeout pr_warn on either box.
    # NOTE: grep -c prints "0" but exits rc=1 on zero matches — the remote
    # echo must live INSIDE the ssh command, else the fallback appends a
    # second line and the -eq checks see "0\n0".
    ko1="$(dmesg 2>/dev/null | grep -c 'dmabuf submit timeout' || true)"
    ko2="$(odl_peer "dmesg | grep -c 'dmabuf submit timeout' 2>/dev/null || echo 0")"
    ko1="${ko1%%$'\n'*}"; ko2="${ko2%%$'\n'*}"
    ko1="${ko1:-0}"; ko2="${ko2:-0}"

    # leak check: in-flight frames must drain back to 0 at both ends
    leak=0
    if [ "$NO_DEBUGFS" = "0" ]; then
        i1=$(odl_get_stat cur_tx_inflight); i2=$(odl_peer_stat cur_tx_inflight)
        if [ "${i1:-0}" != "0" ] || [ "${i2:-0}" != "0" ]; then
            leak=1
            odl_warn "cur_tx_inflight not drained: local=${i1:-?} peer=${i2:-?}"
        fi
    fi

    verdict="pass"
    [ "$rc" -eq 0 ] || verdict="fail"
    [ "$ko1" -eq 0 ] && [ "$ko2" -eq 0 ] || verdict="fail"
    [ "$leak" -eq 0 ] || verdict="fail"

    if [ "$verdict" = "pass" ]; then
        pass=$((pass+1))
        printf 't=%03d sz=%s rc=%d timeout_s1=%s timeout_s2=%s leak=%d -> PASS\n' \
            "$t" "$sz" "$rc" "$ko1" "$ko2" "$leak" | tee -a "$FARM/summary.txt"
        rm -f "$FARM/trial-$t-client.tmp" "$FARM/trial-$t-server.out"
    else
        fail=$((fail+1))
        [ "$rc" -eq 124 ] && hard=$((hard+1))
        mkdir -p "$FARM/fail"
        mv "$FARM/trial-$t-client.tmp" "$FARM/fail/t$t-client.log" 2>/dev/null
        mv "$FARM/trial-$t-server.out" "$FARM/fail/t$t-server.log" 2>/dev/null
        odl_forensics "t$t"
        # the submitted-vs-completed delta is the leak measure (Y − X)
        if [ "$NO_DEBUGFS" = "0" ]; then
            sb=$(odl_get_stat tx_frames_submitted); cp_=$(odl_get_stat tx_frames_completed)
            sb="${sb:-0}"; cp_="${cp_:-0}"
            printf 't%s submitted=%s completed=%s delta=%d\n' \
                "$t" "$sb" "$cp_" "$(( sb - cp_ ))" >> "$FARM/summary.txt"
        fi
        printf 't=%03d sz=%s rc=%d hard=%s timeout_s1=%d timeout_s2=%d leak=%d -> FAIL (in fail/)\n' \
            "$t" "$sz" "$rc" "$([ "$rc" -eq 124 ] && echo yes || echo no)" "$ko1" "$ko2" "$leak" \
            | tee -a "$FARM/summary.txt"
        if [ "${ODL_STOP_ON_FAIL:-0}" = "1" ]; then break; fi
    fi
done

printf '=== done: pass=%d fail=%d hard=%d (farm: %s)\n' "$pass" "$fail" "$hard" "$FARM" \
    | tee -a "$FARM/summary.txt"

if [ "$fail" -eq 0 ]; then
    odl_log "gate-dmabuf: PASS ($pass/$TRIALS)"
    exit 0
fi
odl_log "gate-dmabuf: FAIL ($pass/$TRIALS, $fail failures, $hard hard)"
exit 1