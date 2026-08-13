#!/usr/bin/env bash
#
# odl-gate-sweep — Tier 3b: the wide size × stream matrix.
#
# Cheap once the point gates exist: a loop over block sizes and stream
# counts, driven entirely by odl_tb5_cli (machine-readable CSV and
# comma-separated sizes are native).  Runs each combination to
# completion and STOPS ON FAIL by default, matching the source suite's
# --stop-on-fail positioning ("worth having before a nontrivial handoff").
#
# One OdinLink-specific expectation encoded here: a dmabuf request
# larger than the ring capacity returns -ENOSPC *up front* (the F2
# capacity pre-check) — an expected answer for an oversized request, not
# a regression.  The CLI stream path handles the same sizes natively, so
# the matrix above uses only in-ring sizes unless ODL_INCLUDE_OVERSIZE=1.
#
# Usage:
#   odl-gate-sweep.sh [--sizes LIST] [--streams LIST] [--dur S]
#   --sizes LIST    comma- or space-separated sizes (default "4K 64K 1M 4M")
#   --streams LIST  stream counts (default "1 4")
#   --dur S         seconds per run (default ODL_LOAD_SECS, 10)
#   --no-stop       continue on failure instead of stopping
#
# Exit: 0 all combinations clean; 1 any failed; 2 skipped.

set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$DIR/od/common.sh"

SIZES="${ODL_SIZES:-4K 64K 1M 4M}"
STREAMS="${ODL_STREAMS_LIST:-1 4}"
DUR="${ODL_DUR:-10}"
STOP="${ODL_STOP_ON_FAIL:-1}"

while [ $# -gt 0 ]; do
    case "$1" in
        --sizes)    SIZES="$(echo "$2" | tr ',' ' ')"; shift 2 ;;
        --streams)  STREAMS="$(echo "$2" | tr ',' ' ')"; shift 2 ;;
        --dur)      DUR="$2"; shift 2 ;;
        --no-stop)  STOP=0; shift ;;
        -h|--help)  sed -n '2,36p' "$0"; exit 0 ;;
        *) odl_die "unknown arg: $1" ;;
    esac
done

odl_setup "gate-sweep"
[ -x "$ODL_CLI" ] || odl_skip "cli binary missing: $ODL_CLI"

odl_log "sweep sizes={$SIZES} streams={$STREAMS} duration=${DUR}s"

fails=0; total=0

for ns in $STREAMS; do
    for sz in $SIZES; do
        total=$((total+1))
        unit="odl-swep-$ns-$sz-$$"
        systemd-run --unit="$unit" --collect --no-block \
            env LD_LIBRARY_PATH="$ODL_LD_LIBRARY_PATH" \
            "$ODL_CLI" server -d "${ODL_DEV:-0}" \
            >"$FARM/s-$sz-n$ns-server.out" 2>&1 </dev/null || odl_die "server start failed"
        sleep 1

        if [ "$ns" -gt 1 ]; then
            ctest="mimo --streams $ns"
        else
            ctest="bandwidth"
        fi

        # shellcheck disable=SC2086
        ssh $ODL_SSH_OPTS "$ODL_PEER" \
            "env LD_LIBRARY_PATH=$ODL_LD_LIBRARY_PATH \
             $ODL_CLI client -t $ctest -b $sz -D $DUR -d ${ODL_DEV:-0}" \
            >"$FARM/sweep-$sz-n$ns-client.out" 2>&1
        rc=$?

        systemctl stop "$unit" 2>/dev/null || true
        systemctl reset-failed "$unit" 2>/dev/null || true

        # Human scale: the measured throughput, not the link speed.  Anchor
        # on the "Throughput:" label — the client also prints a "Link speed:
        # N Gb/s" banner line ABOVE it, and an unanchored match reported that
        # banner for every cell of the matrix (identical number in every row,
        # which is the tell that a gate is not measuring anything).
        gbps="$(grep -m1 -oE 'Throughput:[[:space:]]+[0-9.]+ Gb/s' \
                "$FARM/sweep-$sz-n$ns-client.out" 2>/dev/null |
                grep -oE '[0-9.]+ Gb/s' | head -1)"

        if [ "$rc" -eq 0 ]; then
            printf 'size=%s streams=%s rc=0 throughput=%s -> PASS\n' \
                "$sz" "$ns" "${gbps:-n/a}" | tee -a "$FARM/summary.txt"
        else
            fails=$((fails+1))
            mkdir -p "$FARM/fail"
            mv "$FARM/sweep-$sz-n$ns-client.out" "$FARM/fail/sweep-${sz}-n${ns}-client.log" 2>/dev/null
            mv "$FARM/sweep-$sz-n$ns-server.out" "$FARM/fail/sweep-${sz}-n${ns}-server.log" 2>/dev/null
            printf 'size=%s streams=%s rc=%d - FAIL (in fail/)\n' \
                "$sz" "$ns" "$rc" | tee -a "$FARM/summary.txt"
            odl_forensics "sweep-${sz}-n${ns}"
            [ "$STOP" = "1" ] && { odl_log "matrix stop-on-failure after $sz/$ns"; break 2; }
        fi
    done
done

printf 'sweep: %d combinations, %d passed, %d failed\n' "$total" "$((total-fails))" "$fails" \
    | tee -a "$FARM/summary.txt"

if [ "$fails" -eq 0 ]; then
    odl_log "gate-sweep: PASS"
    exit 0
fi
odl_log "gate-sweep: FAIL ($fails)"
exit 1