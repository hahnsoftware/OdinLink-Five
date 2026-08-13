#!/usr/bin/env bash
#
# odl-gate-integrity — Tier 1b: stream-path payload verification.
#
# The only defence against silent corruption on the CLI/stream path,
# which otherwise has ZERO byte checking (bandwidth/latency/jitter/mimo
# all measure timing and counts, never a payload byte).  Wraps
# tests/odl_stream_verify (the rc_write_verify analogue for OdinLink).
#
# What is checked (see odl_stream_verify.c):
#   * fragment reassembly under the 8-byte stream header — the
#     reorder/duplicate/misattribute class where every counter stays
#     clean while bytes are wrong
#   * multi-path ordering hazards and multi-stream concurrency
#   * non-delivery (receiver re-poisons its buffer every round)
#   * BOTH directions — the client verifies the server's echo leg and
#     the server verifies its own received leg, so TX and RX differ
#     across all four legs of a round trip
#
# Usage:
#   odl-gate-integrity.sh [--trials N] [--streams N] [--rounds N]
#                          [--size S] [--timeout S] [--no-debugfs]
#   --trials N    repeat the whole verify run N times, report passes/N
#                 (default 1; liveness-sensitive settings raise this)
#   --streams N   concurrent streams (default 4, mirrors MIMO ids 20..23)
#   --rounds N    echo rounds per stream (default 8)
#   --size S      payload size, K/M/G (default 1M → ~256 fragment
#                 reassemblies per round)
#   --timeout S   per-trial wall clock (default ODL_TRIAL_TIMEOUT, 300)
#   --no-debugfs  skip the counter+leak sanity wrappers
#
# Exit: 0 all trials clean; 1 any corruption/error; 2 skipped.

set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$DIR/od/common.sh"

TRIALS="${ODL_TRIALS:-1}"
STREAMS="${ODL_STREAMS:-4}"
ROUNDS="${ODL_ROUNDS:-8}"
SIZE="${ODL_SIZE:-1M}"
TRIAL_TIMEOUT="${ODL_TRIAL_TIMEOUT:-300}"
NO_DEBUGFS="${ODL_NO_DEBUGFS:-0}"

while [ $# -gt 0 ]; do
    case "$1" in
        --trials)  TRIALS="$2"; shift 2 ;;
        --streams) STREAMS="$2"; shift 2 ;;
        --rounds)  ROUNDS="$2"; shift 2 ;;
        --size)    SIZE="$2"; shift 2 ;;
        --timeout) TRIAL_TIMEOUT="$2"; shift 2 ;;
        --no-debugfs) NO_DEBUGFS=1; shift ;;
        -h|--help) sed -n '2,36p' "$0"; exit 0 ;;
        *) odl_die "unknown arg: $1" ;;
    esac
done

odl_setup "gate-integrity"
[ -x "$ODL_STREAM_VERIFY" ] || odl_skip "stream-verify binary missing: $ODL_STREAM_VERIFY"

pass=0; fail=0

for t in $(seq 1 "$TRIALS"); do
    unit="odl-stv-$t-$$"
    systemd-run --unit="$unit" --collect --no-block \
        env LD_LIBRARY_PATH="$ODL_LD_LIBRARY_PATH" \
        "$ODL_STREAM_VERIFY" server --streams "$STREAMS" --rounds "$ROUNDS" \
            --size "$SIZE" \
        >"$FARM/trial-$t-server.out" 2>&1 </dev/null || odl_die "server start failed"
    sleep 2

    # shellcheck disable=SC2086
    ssh $ODL_SSH_OPTS "$ODL_PEER" \
        "env LD_LIBRARY_PATH=$ODL_LD_LIBRARY_PATH timeout $TRIAL_TIMEOUT \
         $ODL_STREAM_VERIFY client --streams $STREAMS --rounds $ROUNDS \
            --size $SIZE" \
        >"$FARM/trial-$t-client.tmp" 2>&1
    rc=$?

    systemctl stop "$unit" 2>/dev/null || true
    systemctl reset-failed "$unit" 2>/dev/null || true

    # a failed trial is anything the binary itself flags OR anything we
    # can see from the outside (hang, shadow of a messy transfer).
    # Word-boundary-guarded: the client's own summary prints "shorts=0
    # errs=0 bad=0", and a bare SHORT/ERROR match would trip on those.
    verdict="pass"
    [ "$rc" -eq 0 ] || verdict="fail"
    grep -qiE 'INTEGRITY FAIL|CORRUPT|(^|[^[:alnum:]])SHORT([^[:alnum:]]|$)|(^|[^[:alnum:]])ERROR([^[:alnum:]]|$)' \
        "$FARM/trial-$t-client.tmp" && verdict="fail"
    grep -qiE 'CORRUPT|(^|[^[:alnum:]])SHORT([^[:alnum:]]|$)|(^|[^[:alnum:]])ERROR([^[:alnum:]]|$)' \
        "$FARM/trial-$t-server.out" && verdict="fail"

    if [ "$verdict" = "pass" ]; then
        pass=$((pass+1))
        printf 't=%03d streams=%s rounds=%s size=%s rc=%d -> PASS\n' \
            "$t" "$STREAMS" "$ROUNDS" "$SIZE" "$rc" | tee -a "$FARM/summary.txt"
        rm -f "$FARM/trial-$t-client.tmp" "$FARM/trial-$t-server.out"
    else
        fail=$((fail+1))
        mkdir -p "$FARM/fail"
        mv "$FARM/trial-$t-client.tmp" "$FARM/fail/t$t-client.log" 2>/dev/null
        mv "$FARM/trial-$t-server.out" "$FARM/fail/t$t-server.log" 2>/dev/null
        odl_forensics "t$t"
        printf 't=%03d streams=%s rounds=%s size=%s rc=%d -> FAIL (in fail/)\n' \
            "$t" "$STREAMS" "$ROUNDS" "$SIZE" "$rc" | tee -a "$FARM/summary.txt"
        if [ "${ODL_STOP_ON_FAIL:-0}" = "1" ]; then break; fi
    fi
done

odl_report "$fail" "$pass" "gate-integrity (${STREAMS}x${ROUNDS}, ${SIZE})"
rc=$?
printf "farm: %s\n" "$FARM"
exit "$rc"