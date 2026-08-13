#!/usr/bin/env bash
#
# odl-link-check — Tier 0a gate: assert a clean, agreed link before any
# measurement.
#
# As a gate it checks, on BOTH boxes:
#   * cur_state == 4 (READY)
#   * cur_negotiated_paths is agreed (> 0, equal, or == ODL_EXPECTED_PATHS)
#   * the drop / reassembly-fault counter family is zero at start
#   * optionally (ODL_EXPECT_SPEED_GBPS set) that the negotiated link
#     speed reaches the expected threshold, via one short CLI exchange
#
# Dot-source it ("source tools/gates/odl-link-check.sh") from other gates
# and call `odl_link_check` to protect them against a mistrained link —
# the principal source of phantom regressions on this rig.
#
# Usage:
#   ODL_PEER=root@<peer-host> ODL_BUILD_DIR=/root/odl/build ./odl-link-check.sh
# Env:
#   ODL_PEER            peer host (user@host)
#   ODL_EXPECTED_PATHS  require this cur_negotiated_paths (default: any >0)
#   ODL_LINK_SPEED_GBPS run a CLI exchange and require >= this (default off)
#   ODL_ALLOW_FAULTS=1  tolerate pre-existing nonzero counters
#
# Exit: 0 pass, 1 fail, 2 skip (debugfs or peer absent).

set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$DIR/od/common.sh"

ODL_EXPECTED_PATHS="${ODL_EXPECTED_PATHS:-}"
ODL_LINK_SPEED_GBPS="${ODL_LINK_SPEED_GBPS:-}"

# Fault-family counters that must be zero at the START of a clean run.
# Names are the exact debugfs keys from driver/odl_tb5_core.h
# (ODL_TB5_STATS_FIELDS).
_ODL_FAULT_KEYS="tx_frames_canceled rx_frames_canceled rx_frames_runt
rx_asm_reset_incomplete rx_asm_append_skipped rx_asm_cap_exceeded rx_asm_grow_fail
rx_msgs_drop_overflow rx_msgs_drop_alloc rx_repost_pool_empty rx_repost_ring_fail"

odl_link_check ()
{
    odl_require_debugfs || return 2

    local bad=0 k v vp

    # 1) state == READY + path agreement
    local s_local s_peer p_local p_peer
    s_local="$(odl_get_stat cur_state)" ;        s_peer="$(odl_peer_stat cur_state)"
    p_local="$(odl_get_stat cur_negotiated_paths)"; p_peer="$(odl_peer_stat cur_negotiated_paths)"
    odl_log "cur_state local=${s_local:-?} peer=${s_peer:-?} (4=READY); paths local=${p_local:-?} peer=${p_peer:-?}"

    [ "${s_local:-X}" = "4" ] || { odl_log "FAIL local not READY (state ${s_local:-?})"; bad=1; }
    [ "${s_peer:-X}" = "4" ]  || { odl_log "FAIL peer not READY (state ${s_peer:-?})"; bad=1; }
    [ "${p_local:-0}" -gt 0 ] || { odl_log "FAIL local path count ${p_local:-?}"; bad=1; }
    [ "${p_peer:-0}" -gt 0 ]  || { odl_log "FAIL peer path count ${p_peer:-?}"; bad=1; }
    if [ -n "$ODL_EXPECTED_PATHS" ] && [ "${p_local:-0}" != "$ODL_EXPECTED_PATHS" ]; then
        odl_log "FAIL expected $ODL_EXPECTED_PATHS paths, got ${p_local:-?}"; bad=1
    fi
    if [ "$p_local" != "$p_peer" ]; then
        odl_log "FAIL boxes disagree on negotiated paths"; bad=1
    fi

    # 2) pre-existing fault counters
    for k in $_ODL_FAULT_KEYS; do
        v="$(odl_get_stat "$k")"; vp="$(odl_peer_stat "$k")"
        if [ "${v:-0}" != "0" ]; then
            odl_warn "local counter not zero at start: $k=$v"; bad=1
        fi
        if [ "${vp:-0}" != "0" ]; then
            odl_warn "peer counter not zero at start: $k=$vp"; bad=1
        fi
    done

    # 3) optional live link-speed check
    if [ -n "$ODL_LINK_SPEED_GBPS" ] && [ "$bad" -eq 0 ]; then
        odl_link_check_speed "$ODL_LINK_SPEED_GBPS" || bad=1
    fi

    if [ "$bad" -eq 0 ]; then
        odl_log "odl-link-check: PASS"
        return 0
    fi
    odl_log "odl-link-check: FAIL"
    return 1
}

# One short CLI exchange: bring up a server on THIS box, have the peer
# blast a 50-iteration latency test, scrape the "Link speed: N Gb/s (xM
# lanes)" line out of either end's output, compare to $1.
odl_link_check_speed ()
{
    local want="$1"
    local srv="$FARM/link-speed.out" slowbest
    rm -f "$srv"

    ODL_SRV_PID= ; ODL_SRV_UNIT=
    if command -v systemd-run >/dev/null 2>&1 && \
            systemctl is-system-running >/dev/null 2>&1; then
        ODL_SRV_UNIT="odl-linkspd-$PPID"
        systemd-run --unit="$ODL_SRV_UNIT" --collect --no-block \
            env LD_LIBRARY_PATH="$ODL_LD_LIBRARY_PATH" \
            "$ODL_CLI" server -d "${ODL_DEV:-0}" >"$srv" 2>&1
    else
        # nohup style (setsid so a normal exit of the caller does not kill)
        setsid env LD_LIBRARY_PATH="$ODL_LD_LIBRARY_PATH" \
            "$ODL_CLI" server -d "${ODL_DEV:-0}" >"$srv" 2>&1 </dev/null &
        ODL_SRV_PID=$!
    fi
    sleep 2

    # shellcheck disable=SC2086
    ssh $ODL_SSH_OPTS "$ODL_PEER" \
        "env LD_LIBRARY_PATH=$ODL_LD_LIBRARY_PATH timeout 30 \
         $ODL_CLI client -t latency -i 50 -d ${ODL_DEV:-0}" \
        >"$FARM/link-speed-peer.out" 2>&1

    if [ -n "$ODL_SRV_UNIT" ]; then
        systemctl stop "$ODL_SRV_UNIT" 2>/dev/null || true
        systemctl reset-failed "$ODL_SRV_UNIT" 2>/dev/null || true
    fi
    [ -n "$ODL_SRV_PID" ] && kill "$ODL_SRV_PID" 2>/dev/null || true

    local cap
    cap="$(grep -m1 -oE 'Link speed: [0-9]+ Gb/s' "$FARM/link-speed-peer.out" 2>/dev/null | awk '{print $3}')"
    [ -n "$cap" ] || cap="$(grep -m1 -oE 'Link speed: [0-9]+ Gb/s' "$srv" 2>/dev/null | awk '{print $3}')"
    if [ -z "$cap" ]; then
        odl_log "link-speed probe: could not read negotiated speed"
        return 1
    fi
    odl_log "negotiated link speed ${cap} Gb/s (want ≥ ${want})"
    if [ "$cap" -lt "$want" ]; then
        odl_log "link below expected speed (got $cap < $want) — cable/retimer?"
        return 1
    fi
    return 0
}

# ---- direct invocation gates --------------------------------------------
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    odl_setup "${ODL_GATE_NAME:-link-check}"
    odl_link_check
    case "$?" in
        0) exit 0 ;;
        2) exit 2 ;;
        *) exit 1 ;;
    esac
fi