#!/usr/bin/env bash
#
# odl-gate-frame-balance — Tier 2b: frame conservation across the link.
#
# The idea transfers almost directly from `thunderbolt-ibverbs gate-q8`:
# a transport that DROPS frames and re-sends them hides loss as latency
# and passes every bandwidth test.  This gate reconciles, per path, the
# frames a sender completed against the frames the far END received,
# after a quiesce.  A shortfall means a frame left the host and never
# landed.
#
# Per path, both directions are compared (each end sends AND receives
# during an echo workload):
#     local_pN_tx_frames  vs  peer_pN_rx_frames
#     peer_pN_tx_frames   vs  local_pN_rx_frames
# Residual may not exceed ODL_FRAME_TOL after polling-quiesce — not a
# fixed sleep, because cur_tx_inflight demonstrably takes time to
# settle (and, as the F2 sessions showed, sometimes never does).
#
# Also reports and can gate the rx_frames_no_stream asymmetry — the
# counter that sat at 3,076,123 on one box and 0 on the other during
# the F2 session, with nobody watching a threshold on it.
#
# Usage:
#   odl-gate-frame-balance.sh [--load cli|bench] [--load-secs S] [--tol N]
#                              [--no-stream-tol N] [--quiesce-max S]
#   --load cli      drive with odl_tb5_cli (stream path; path-0 intense)
#   --load bench    drive with odl_tb5_bench_dmabuf (zero-copy, striped,
#                   exercises per-path frame counters across all paths)
#   --load-secs S   CLI workload duration (default 20)
#   --tol N         per-path reconciliation tolerance, frames (default 8)
#   --no-stream-tol N  max |local - peer| rx_frames_no_stream (default 100;
#                   0 disables the check)
#   --quiesce-max S polling window for counter stability (default 60)
#
# Env: ODL_LOAD, ODL_TRIAL_TIMEOUT, plus the common rig env.
#
# Exit: 0 balanced; 1 shortfall / asymmetry; 2 skipped (debugfs/tool).

set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$DIR/od/common.sh"

LOAD="${ODL_LOAD:-cli}"
LOAD_SECS="${ODL_LOAD_SECS:-20}"
FRAME_TOL="${ODL_FRAME_TOL:-8}"
NO_STREAM_TOL="${ODL_NO_STREAM_TOL:-100}"
QUIESCE_MAX="${ODL_QUIESCE_MAX:-60}"
TRIAL_TIMEOUT="${ODL_TRIAL_TIMEOUT:-180}"

while [ $# -gt 0 ]; do
    case "$1" in
        --load)          LOAD="$2"; shift 2 ;;
        --load-secs)     LOAD_SECS="$2"; shift 2 ;;
        --tol)           FRAME_TOL="$2"; shift 2 ;;
        --no-stream-tol) NO_STREAM_TOL="$2"; shift 2 ;;
        --quiesce-max)   QUIESCE_MAX="$2"; shift 2 ;;
        -h|--help) sed -n '2,40p' "$0"; exit 0 ;;
        *) odl_die "unknown arg: $1" ;;
    esac
done

odl_setup "gate-frame-balance"
odl_require_debugfs
if [ "$LOAD" = "bench" ] && [ ! -x "$ODL_BENCH" ]; then
    odl_skip "bench binary missing: $ODL_BENCH"
elif [ "$LOAD" = "cli" ] && [ ! -x "$ODL_CLI" ]; then
    odl_skip "cli binary missing: $ODL_CLI"
elif [ "$LOAD" != "cli" ] && [ "$LOAD" != "bench" ]; then
    odl_die "--load must be cli or bench"
fi

NP="$(odl_get_stat cur_negotiated_paths)"
odl_log "negotiated paths = ${NP:-?}"

# ---- counters snapshot as a canonical line (whole stats file) -----------
grab ()
{
    cat "$ODL_STATS_FILE" 2>/dev/null
}

local_count () { odl_get_stat "$1"; }
peer_count  () { odl_peer_stat "$1"; }

# ---- quiesce: poll both ends until their frame counters stop moving ----
odl_quiesce()
{
    local prev_l prev_p cur_l cur_p stable=0 loops=0
    prev_l="$(grab)"; prev_p="$(odl_peer "cat $ODL_STATS_FILE" 2>/dev/null)"
    while :; do
        sleep 5
        loops=$((loops+5))
        s_l="$(grab)"; s_p="$(odl_peer "cat $ODL_STATS_FILE" 2>/dev/null)"
        if [ "$s_l" = "$prev_l" ] && [ "$s_p" = "$prev_p" ]; then
            stable=$((stable+5))
            [ "$stable" -ge 10 ] && break
        else
            stable=0
        fi
        prev_l="$s_l"; prev_p="$s_p"
        [ "$loops" -ge "$QUIESCE_MAX" ] && break
    done
    odl_log "quiesce reached after ${loops}s (stable ${stable}s)"
}

# ---- reset + run load ----------------------------------------------------
printf '1\n' >"$ODL_STATS_RESET" 2>/dev/null || odl_fail "stats_reset failed"
odl_peer "printf '1\n' > $ODL_STATS_RESET 2>/dev/null || true"

server_bin=""; client_bin=""
if [ "$LOAD" = "cli" ]; then
    server_bin="$ODL_CLI";  client_bin="$ODL_CLI"
    srv=(server)
    client_cmd="client -t mimo -b 1M -D $LOAD_SECS"
else
    server_bin="$ODL_BENCH"; client_bin="$ODL_BENCH"
    srv=(server --iters 200 --warmup 20 --sizes 1M)
    client_cmd="client --iters 200 --warmup 20 --sizes 1M"
fi
odl_server_start "$server_bin" "${srv[@]}"

# shellcheck disable=SC2086
odl_run_timeout "$TRIAL_TIMEOUT" -- ssh $ODL_SSH_OPTS "$ODL_PEER" \
    "env LD_LIBRARY_PATH=$ODL_LD_LIBRARY_PATH $client_bin $client_cmd" \
    >"$FARM/load-client.out" 2>&1
rc=$?
odl_server_stop
if [ "$rc" -ne 0 ]; then
    odl_log "load driver failed (rc=$rc) — see load-client.out; quiescing anyway"
    odl_forensics "loadfail"
fi
odl_log "load done; quiescing..."
odl_quiesce

# ---- snapshots for final reconciliation ---------------------------------
cp "$ODL_STATS_FILE" "$FARM/final-local.stats"
odl_peer_capture "$ODL_STATS_FILE" "$FARM/final-peer.stats"

# ---- per-path reconciliation ----------------------------------------------
absd() { [ "$1" -lt 0 ] && echo $(( -$1 )) || echo "$1"; }

fails=0
NP="${NP:-$(odl_get_stat cur_negotiated_paths)}"
NP="${NP:-2}"
for p in $(seq 0 $((NP-1))); do
    lt="$(grep "^p${p}_tx_frames" "$FARM/final-local.stats" | awk '{print $2}')"
    lr="$(grep "^p${p}_rx_frames" "$FARM/final-local.stats" | awk '{print $2}')"
    pt="$(grep "^p${p}_tx_frames" "$FARM/final-peer.stats" | awk '{print $2}')"
    pr="$(grep "^p${p}_rx_frames" "$FARM/final-peer.stats" | awk '{print $2}')"
    lt="${lt:-0}"; lr="${lr:-0}"; pt="${pt:-0}"; pr="${pr:-0}"

    # local sent vs peer received, and vice versa
    short1=$((lt - pr));  short2=$((pt - lr))
    a1="$(absd "$short1")"; a2="$(absd "$short2")"

    printf 'path p%d tx(l=%s r=%s) peer(tx=%s rx=%s) short=%s/%s\n' \
        "$p" "$lt" "$lr" "$pt" "$pr" "$short1" "$short2" | tee -a "$FARM/summary.txt"

    if [ "$a1" -gt "$FRAME_TOL" ] || [ "$a2" -gt "$FRAME_TOL" ]; then
        odl_log "FAIL path $p unbalanced (tol $FRAME_TOL)"
        fails=$((fails+1))
    fi
done

# ---- the rx_frames_no_stream asymmetry (the 3M-vs-0 counter) ---------------
if [ "$NO_STREAM_TOL" -gt 0 ]; then
    n1="$(local_count rx_frames_no_stream)"; n2="$(peer_count rx_frames_no_stream)"
    n1="${n1:-0}"; n2="${n2:-0}"
    d=$(( n1 - n2 )); [ "$d" -lt 0 ] && d=$(( -d ))
    printf 'rx_frames_no_stream local=%s peer=%s (divergence %s)\n' \
        "$n1" "$n2" "$d" | tee -a "$FARM/summary.txt"
    if [ "$d" -gt "$NO_STREAM_TOL" ]; then
        odl_log "FAIL rx_frames_no_stream divergence $d > $NO_STREAM_TOL"
        fails=$((fails+1))
    fi
fi

# ---- submission bookkeeping (nothing silently cancelled) -------------------
for k in tx_frames_submitted tx_frames_completed tx_frames_canceled; do
    v="$(local_count "$k")"; vp="$(peer_count "$k")"
    printf '  %s local=%s peer=%s\n' "$k" "$v" "$vp" | tee -a "$FARM/summary.txt"
done

if [ "$fails" -eq 0 ]; then
    odl_log "gate-frame-balance: PASS"
    exit 0
fi
odl_log "gate-frame-balance: FAIL $fails"
exit 1