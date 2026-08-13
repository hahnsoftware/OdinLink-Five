#!/usr/bin/env bash
#
# odl-gate-latency — Tier 2c: tail latency + mode occupancy.
#
# The rig is bimodal: ~11 µs and ~22 µs medians, selected per run and
# occasionally mid-run, so ANY latency gate that keys on a single median
# flaps between "25 % faster" and "25 % slower" on mode selection alone.
# This gate therefore:
#
#   * runs many repeats (default 30) and reports the DISTRIBUTION of
#     p99.9/max across runs — the tail, which is far less mode-sensitive
#   * reports MODE OCCUPANCY as a first-class output: the fraction of
#     samples below the fast/slow split (ODL_MODE_SPLIT_US, default 15)
#     — a shift in mode occupancy is the real signal
#   * gates on the tail (p99.9), never on a single median
#   * checks the MECHANISM as well as the number: odl_busy_poll_us at
#     its intended value and cur_poll_active behaving (a config drift
#     cannot masquerade as a performance change)
#
# The bimodality itself is unexplained; until it is, treat every median
# from this rig as unquotable — but the tail is gatable.
#
# Usage:
#   odl-gate-latency.sh [--runs N] [--iters N] [--max-p999-us X]
#                        [--mode-split-us X] [--expect-busy-poll N]
#   --runs N            repeats (default 30; ≥20 recommended)
#   --iters N           iterations per repeat (default 10000)
#   --max-p999-us X     FAIL if median p99.9 > X µs.  UNSET (default) =
#                       report-only: the tail is not a hardware contract
#                       on every target — the source suite's comparable
#                       figure was fast-path firmware, and a plain
#                       USB4/TB rig answers in 10s-of-ms.  Set the
#                       ceiling (env ODL_MAX_P999_US or this flag) to
#                       turn it into a gate.
#   --mode-split-us X   fast/slow split (default 15)
#   --expect-busy-poll N  require odl_busy_poll_us == N (default: unset
#                       -> only report)
#
# Env: ODL_RUNS, ODL_ITERS, ODL_MODE_SPLIT_US, ODL_MAX_P999_US, ODL_PEER,
#      ODL_BUILD_DIR...
#
# Exit: 0 pass; 1 tail/mechanism fail; 2 skipped.

set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$DIR/od/common.sh"

RUNS="${ODL_RUNS:-30}"
ITERS="${ODL_ITERS:-10000}"
MAX_P999="${ODL_MAX_P999_US:-}"
MODE_SPLIT="${ODL_MODE_SPLIT_US:-15}"
EXPECT_BUSY_POLL="${ODL_EXPECT_BUSY_POLL:-}"

while [ $# -gt 0 ]; do
    case "$1" in
        --runs)            RUNS="$2"; shift 2 ;;
        --iters)           ITERS="$2"; shift 2 ;;
        --max-p999-us)     MAX_P999="$2"; shift 2 ;;
        --mode-split-us)   MODE_SPLIT="$2"; shift 2 ;;
        --expect-busy-poll) EXPECT_BUSY_POLL="$2"; shift 2 ;;
        -h|--help) sed -n '2,40p' "$0"; exit 0 ;;
        *) odl_die "unknown arg: $1" ;;
    esac
done

odl_setup "gate-latency"
odl_require_debugfs
[ -x "$ODL_CLI" ] || odl_skip "cli binary missing: $ODL_CLI"

bp="$(odl_modparam odl_busy_poll_us)"
odl_log "odl_busy_poll_us = ${bp:-?} (expect ${EXPECT_BUSY_POLL:-report-only})"
if [ -n "$EXPECT_BUSY_POLL" ] && [ "${bp:-}" != "$EXPECT_BUSY_POLL" ]; then
    odl_log "FAIL odl_busy_poll_us = ${bp:-?}, expected $EXPECT_BUSY_POLL"
    exit 1
fi

MODE_SPLIT_NS=$((MODE_SPLIT * 1000))

: > "$FARM/p999.txt"
: > "$FARM/max.txt"
: > "$FARM/occupancy.txt"
: > "$FARM/poll.txt"

fails=0

for run in $(seq 1 "$RUNS"); do
    # fresh CLI server per run (the server exits after each block)
    unit="odl-lat-$run-$$"
    systemd-run --unit="$unit" --collect --no-block \
        env LD_LIBRARY_PATH="$ODL_LD_LIBRARY_PATH" \
        "$ODL_CLI" server -d "${ODL_DEV:-0}" \
        >"$FARM/run-$run-server.out" 2>&1 </dev/null || odl_die "server start failed"
    sleep 1

    # client on the peer, CSV pulled back
    # shellcheck disable=SC2086
    ssh $ODL_SSH_OPTS "$ODL_PEER" \
        "env LD_LIBRARY_PATH=$ODL_LD_LIBRARY_PATH \
         $ODL_CLI client -t latency -i $ITERS -o /tmp/odl-lat-$run.csv \
            -d ${ODL_DEV:-0}" \
        >"$FARM/run-$run-client.out" 2>&1
    rc=$?

    # sample the poll mechanism mid-run (busy-poll active while loaded?)
    p=$(odl_get_stat cur_poll_active)

    systemctl stop "$unit" 2>/dev/null || true
    systemctl reset-failed "$unit" 2>/dev/null || true

    odl_peer_capture "/tmp/odl-lat-$run.csv" "$FARM/run-$run.csv" || true

    if [ "$rc" -ne 0 ] || [ ! -s "$FARM/run-$run.csv" ]; then
        odl_log "run $run: client rc=$rc, csv missing — FAIL"
        fails=$((fails+1))
        continue
    fi

    p999=$(awk -F, '/^# p999_ns/{print $2}' "$FARM/run-$run.csv")
    pmax=$(awk -F, '/^# max_ns/{print $2}' "$FARM/run-$run.csv")
    pmed=$(awk -F, '/^# median_ns/{print $2}' "$FARM/run-$run.csv")
    occ=$(awk -F, -v s="$MODE_SPLIT_NS" \
        '$1 ~ /^[0-9]+$/ { n++; if ($2 < s) f++ }
         END { if (n) printf "%.1f", 100.0*f/n; else print "?" }' \
        "$FARM/run-$run.csv")

    p999=${p999:-0}; pmax=${pmax:-0}; pmed=${pmed:-0}
    printf '%s\n' "$p999" >> "$FARM/p999.txt"
    printf '%s\n' "$pmax" >> "$FARM/max.txt"
    printf '%s\n' "$occ"  >> "$FARM/occupancy.txt"
    printf '%s\n' "$p"    >> "$FARM/poll.txt"

    printf 'run=%03d median=%s p99.9=%s max=%s fast-mode-occ=%s%% poll_active=%s rc=%d\n' \
        "$run" "$pmed" "$p999" "$pmax" "$occ" "${p:-?}" "$rc" | tee -a "$FARM/summary.txt"
done

# ---- aggregate: distribution of tails across runs --------------------------
med() { sort -n "$1" | awk '{a[NR]=$1} END {print a[int((NR+1)/2)]}'; }

p999_med="$(med "$FARM/p999.txt")"
p999_min="$(sort -n "$FARM/p999.txt" | head -1)"
p999_max="$(sort -n "$FARM/p999.txt" | tail -1)"
occ_med="$(med "$FARM/occupancy.txt")"

printf '\n--- aggregate over %d runs ---\n' "$RUNS" | tee -a "$FARM/summary.txt"
printf 'p99.9: min=%s median=%s max=%s us\n' \
    "$p999_min" "$p999_med" "$p999_max" | tee -a "$FARM/summary.txt"
printf 'fast-mode occupancy: median %s%%\n' "$occ_med" | tee -a "$FARM/summary.txt"
printf 'cur_poll_active during load: %s\n' \
    "$(sort -n "$FARM/poll.txt" | uniq -c | tr -s ' ' | tr '\n' ' ')" \
    | tee -a "$FARM/summary.txt"

# gate on the tail — only when a ceiling is explicitly configured
if [ -n "$MAX_P999" ] && [ "${p999_med:-0}" -gt "$MAX_P999" ]; then
    odl_log "FAIL median p99.9 ${p999_med}us > ${MAX_P999}us"
    fails=$((fails+1))
fi

# mechanistic check: polling engaged whenever busy-poll is configured
if [ "${bp:-0}" != "0" ]; then
    if grep -q '^0$' "$FARM/poll.txt"; then
        odl_log "FAIL cur_poll_active was 0 during load with odl_busy_poll_us=${bp}"
        fails=$((fails+1))
    fi
fi

if [ "$fails" -eq 0 ]; then
    odl_log "gate-latency: PASS (report-only numbers above — mode investigation still open)"
    exit 0
fi
odl_log "gate-latency: FAIL $fails"
exit 1