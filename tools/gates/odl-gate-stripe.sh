#!/usr/bin/env bash
#
# odl-gate-stripe — Tier 3a: dmabuf striping + deep-queue stall monitor
# (the Post-F2 "regression monitor" form, not a live-bug repro).
#
# What it protects:
#   * the multi-path zero-copy path (odl_tb5_send/recv_dmabuf striped
#     over `dmabuf_paths`)
#   * the dmabuf ring separation and the drain-first reclaim under
#     deep-queue / ring-full conditions
#   * the depth axis — the source suite's key insight is that the stall
#     class only shows up at high TX depth
#
# Reuses the gate-dmabuf trial machinery (ring-full + next-transfer
# sizes, passes/N, first-failure forensics) for each dmabuf stripe
# width in turn.  drmabuf_paths is WRITABLE (0644) so the width sweep
# needs no reload — set identically on both boxes, measured, restored.
#
# A build whose loaded num_paths is < 2 cannot stripe at all; rather
# than reboot (rmmod is forbidden), the gate SKIPs with a pointer to
# odl-reload-module.sh.  Inspect-and-skip.
#
# Usage:
#   odl-gate-stripe.sh [--widths "1 2"] [--trials N] [--sizes a,b,c]
#   --widths LIST    dmabuf_paths widths to sweep (default "1 $NP")
#   --trials N       rate trials per width (default ODL_TRIALS, 20)
#   --sizes CSV      bench in-ring transfer sizes (default 1048576)
#   --expect-reject CSV  oversized client probe armed before the transfer
#                       loop (default 16777216 — the F2 trigger shape)
#
# Exit: 0 all widths clean; 1 any width failed; 2 skipped (num_paths<2).

set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$DIR/od/common.sh"

TRIALS="${ODL_TRIALS:-20}"
SIZES="1048576"
EXPECT_REJECT="16777216"
ITERS=3
WARMUP=1
TRIAL_TIMEOUT="${ODL_TRIAL_TIMEOUT:-300}"

NP="$(odl_modparam num_paths)"
NP="${NP:-0}"

while [ $# -gt 0 ]; do
    case "$1" in
        --widths)        WIDTHS="$2"; shift 2 ;;
        --trials)        TRIALS="$2"; shift 2 ;;
        --sizes)         SIZES="$2"; shift 2 ;;
        --expect-reject) EXPECT_REJECT="$2"; shift 2 ;;
        --iters)         ITERS="$2"; shift 2 ;;
        --warmup)        WARMUP="$2"; shift 2 ;;
        --timeout)       TRIAL_TIMEOUT="$2"; shift 2 ;;
        -h|--help) sed -n '2,42p' "$0"; exit 0 ;;
        *) odl_die "unknown arg: $1" ;;
    esac
done

odl_setup "gate-stripe"
odl_require_debugfs
[ -x "$ODL_BENCH" ] || odl_skip "bench binary missing: $ODL_BENCH"

[ "$NP" -ge 2 ] || odl_skip "loaded num_paths=$NP < 2 — needs a num_paths=2 load (run odl-reload-module.sh)"

WIDTHS="${WIDTHS:-1 $NP}"
odl_log "stripe sweep: widths {$WIDTHS}, trials $TRIALS, sizes $SIZES"

overall=0

for w in $WIDTHS; do
    [ "$w" -ge 1 ] && [ "$w" -le "$NP" ] || { odl_log "width $w out of range (num_paths=$NP)"; continue; }

    # writable param: set on BOTH ends (must match or the stripe desyncs)
    if [ -w "$ODL_MODPARAM/dmabuf_paths" ]; then
        printf '%s\n' "$w" > "$ODL_MODPARAM/dmabuf_paths" || odl_die "set dmabuf_paths failed"
        odl_peer "echo $w > $ODL_MODPARAM/dmabuf_paths 2>/dev/null || true" || true
    else
        odl_log "dmabuf_paths not writable — SKIP width $w"
        continue
    fi
    sleep 1
    read_dp="$(odl_modparam dmabuf_paths)"
    odl_log "== width dmabuf_paths=$read_dp =="

    pass=0; fail=0
    for t in $(seq 1 "$TRIALS"); do
        unit="odl-strp-$w-$t-$$"

        printf '1\n' >"$ODL_STATS_RESET" 2>/dev/null || true
        odl_peer "printf '1\n' > $ODL_STATS_RESET 2>/dev/null || true" 2>/dev/null || true
        odl_peer "dmesg -c >/dev/null 2>&1 || true" 2>/dev/null || true
        dmesg -c >/dev/null 2>&1 || true

        systemd-run --unit="$unit" --collect --no-block \
            env LD_LIBRARY_PATH="$ODL_LD_LIBRARY_PATH" \
            "$ODL_BENCH" server --iters "$ITERS" --warmup "$WARMUP" \
                --sizes "$SIZES" --expect-reject "$EXPECT_REJECT" \
            >"$FARM/w$w-t$t-server.out" 2>&1 </dev/null || odl_die "server start failed"
        sleep 2

        # shellcheck disable=SC2086
        ssh $ODL_SSH_OPTS "$ODL_PEER" \
            "env LD_LIBRARY_PATH=$ODL_LD_LIBRARY_PATH timeout $TRIAL_TIMEOUT \
             $ODL_BENCH client --iters $ITERS --warmup $WARMUP \
                --sizes $SIZES --expect-reject $EXPECT_REJECT" \
            >"$FARM/w$w-t$t-client.tmp" 2>&1
        rc=$?

        systemctl stop "$unit" 2>/dev/null || true
        systemctl reset-failed "$unit" 2>/dev/null || true

        # grep -c prints "0" but exits rc=1 on zero matches — the remote
        # echo must live INSIDE the ssh command (same trap as gate-dmabuf)
        ko1="$(dmesg 2>/dev/null | grep -c 'dmabuf submit timeout' || true)"
        ko2="$(odl_peer "dmesg | grep -c 'dmabuf submit timeout' 2>/dev/null || echo 0")"
        ko1="${ko1%%$'\n'*}"; ko2="${ko2%%$'\n'*}"
        ko1="${ko1:-0}"; ko2="${ko2:-0}"
        i1=$(odl_get_stat cur_tx_inflight); i2=$(odl_peer_stat cur_tx_inflight)

        verdict="pass"
        [ "$rc" -eq 0 ] || verdict="fail"
        [ "$ko1" -eq 0 ] && [ "$ko2" -eq 0 ] || verdict="fail"
        if [ "${i1:-0}" != "0" ] || [ "${i2:-0}" != "0" ]; then verdict="fail"; fi

        if [ "$verdict" = "pass" ]; then
            pass=$((pass+1))
            rm -f "$FARM/w$w-t$t-client.tmp" "$FARM/w$w-t$t-server.out"
        else
            fail=$((fail+1))
            mkdir -p "$FARM/fail"
            mv "$FARM/w$w-t$t-client.tmp" "$FARM/fail/w${w}-t$t-client.log" 2>/dev/null
            mv "$FARM/w$w-t$t-server.out" "$FARM/fail/w${w}-t$t-server.log" 2>/dev/null
            odl_forensics "w${w}-t$t"
            printf 'width=%s t=%03d rc=%d hard=%s timeout_s1=%s timeout_s2=%s -> FAIL\n' \
                "$w" "$t" "$rc" "$([ "$rc" -eq 124 ] && echo y || echo n)" "$ko1" "$ko2" \
                | tee -a "$FARM/summary.txt"
            [ "${ODL_STOP_ON_FAIL:-0}" = "1" ] && break
        fi
    done

    printf 'width %s: passes=%d/%d fails=%d\n' "$w" "$pass" "$((pass+fail))" "$fail" \
        | tee -a "$FARM/summary.txt"
    overall=$((overall + fail))
done

# restore default dmabuf_paths (0 = negotiated)
if [ -w "$ODL_MODPARAM/dmabuf_paths" ]; then
    printf '0\n' > "$ODL_MODPARAM/dmabuf_paths" 2>/dev/null || true
    odl_peer "echo 0 > $ODL_MODPARAM/dmabuf_paths 2>/dev/null || true" 2>/dev/null || true
fi

if [ "$overall" -eq 0 ]; then
    odl_log "gate-stripe: PASS"
    exit 0
fi
odl_log "gate-stripe: FAIL ($overall failed trials)"
exit 1