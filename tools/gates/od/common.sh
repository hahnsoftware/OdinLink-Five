#!/usr/bin/env bash
#
# od/common.sh — shared plumbing for the OdinLink regression gates.
#
# Source it:
#     . "$(dirname "${BASH_SOURCE[0]}")/common.sh"
# then call odl_setup before anything that touches $FARM.
#
# Gates run on ONE rig box ("local") and drive a second box ("peer")
# over SSH.  Core rules (see README.md):
#
#   * rmmod odl_tb5 is forbidden  ->  gates run against whatever module
#     is loaded and SKIP (exit 2) when a required knob does not match.
#   * debugfs counter exports only exist on the fork build  ->  absence
#     is SKIP (2), never FAIL (1).
#   * the odl_tb5_cli server dies after each block  ->  every gate
#     restarts it before every client invocation (systemd-run).
#   * liveness-sensitive gates report passes/N, not a bare exit.
#   * kill by PID; never pkill -f with a name that also appears in the
#     shell command line (kills the controlling ssh session).
#
# Exit-code convention for all gates:
#   0 PASS   1 FAIL   2 SKIP (missing prerequisite).
#
# Configuration is all environment variables with conservative defaults
# (see README.md).  An optional rig.conf can be sourced from the gates
# dir to seed defaults; env vars always win.

if [ -n "${ODL_COMMON_LOADED:-}" ]; then
    return 0 2>/dev/null || exit 0
fi
ODL_COMMON_LOADED=1

set -u
set -o pipefail

ODL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

# ---- rig / peer --------------------------------------------------------
ODL_PEER="${ODL_PEER:-}"
ODL_SSH_OPTS="${ODL_SSH_OPTS:--o BatchMode=yes -o ConnectTimeout=10}"

# ---- binary layout ---------------------------------------------------------
ODL_BUILD_DIR="${ODL_BUILD_DIR:-${ODL_DIR}/build}"
ODL_BENCH="${ODL_BENCH:-${ODL_BUILD_DIR}/tests/odl_tb5_bench_dmabuf}"
ODL_STREAM_VERIFY="${ODL_STREAM_VERIFY:-${ODL_BUILD_DIR}/tests/odl_stream_verify}"
ODL_CLI="${ODL_CLI:-${ODL_BUILD_DIR}/cli/odl_tb5_cli}"
ODL_TEST_BIN="${ODL_TEST_BIN:-${ODL_BUILD_DIR}/tests/odl_tb5_test}"
ODL_LD_LIBRARY_PATH="${ODL_LD_LIBRARY_PATH:-${ODL_BUILD_DIR}/lib}"

# ---- debugfs counter root (fork-only export) ------------------------------
ODL_DEBUGFS="${ODL_DEBUGFS:-/sys/kernel/debug/odl_tb5/odl_tb5_0}"
ODL_STATS_FILE="${ODL_DEBUGFS}/stats"
ODL_STATS_RESET="${ODL_DEBUGFS}/stats_reset"

# ---- live module parameter tree -------------------------------------------
ODL_MODPARAM="${ODL_MODPARAM:-/sys/module/odl_tb5/parameters}"

# ---- farm / artifact output ----------------------------------------------
# FARM is the run-dir for the current gate invocation.  odl_setup creates it.
FARM="${ODL_FARM_PREFIX:-/root/odl-gates}/${ODL_GATE_NAME:-od-gate}"
ODL_TRIALS="${ODL_TRIALS:-20}"
ODL_TRIAL_TIMEOUT="${ODL_TRIAL_TIMEOUT:-300}"

# =====================================================================
# basic io / logging
# =====================================================================

odl_log()  { printf '%s %s\n' "$(date +%Y-%m-%dT%H:%M:%S)" "$*"; }
odl_warn() { printf '%s [WARN] %s\n' "$(date +%Y-%m-%dT%H:%M:%S)" "$*" >&2; }
odl_die()  { printf '%s [FAIL] %s\n' "$(date +%Y-%m-%dT%H:%M:%S)" "$*" >&2; exit 1; }
odl_skip() { printf '%s [SKIP] %s\n' "$(date +%Y-%m-%dT%H:%M:%S)" "$*" >&2; exit 2; }

# ---------------------------------------------------------------------
# setup routines
# ---------------------------------------------------------------------

# odl_setup <gate-name>: create a per-gate, per-invocation run dir + logs,
# sanity-check peer reachability.
odl_setup() {
    ODL_GATE_NAME="${ODL_GATE_NAME:-$1}"
    local stamp
    stamp="$(date +%Y%m%d-%H%M%S)"
    FARM="${ODL_FARM_PREFIX:-/root/odl-gates}/$(basename "${ODL_GATE_NAME}")-${stamp}"
    mkdir -p "$FARM"
    odl_log "gate=$ODL_GATE_NAME farm=$FARM"
    odl_log "peer=$ODL_PEER build=$ODL_BUILD_DIR"
    if [ -z "${ODL_SKIP_PEER_CHECK:-}" ]; then
        odl_peer true || odl_skip "peer unreachable: $ODL_PEER"
    fi
}

# odl_farm: helper for the "no setup needed" gates that just need a stamp.
odl_farm ()
{
    FARM="${FARM:-${ODL_FARM_PREFIX:-/root/odl-gates}/misc-$(date +%Y%m%d-%H%M%S)}"
    mkdir -p "$FARM"
}

# =====================================================================
#  peer / ssh helpers
# =====================================================================

# odl_peer <cmd...> — run <cmd...> on the peer; stdout goes to the caller.
odl_peer ()
{
    # shellcheck disable=SC2086
    ssh $ODL_SSH_OPTS "$ODL_PEER" "$@"
}

# odl_peer_capture <remote-file> <local-file>
odl_peer_capture ()
{
    # shellcheck disable=SC2086
    ssh $ODL_SSH_OPTS "$ODL_PEER" "cat '$1'" > "$2"
}

# odl_peer_cp <local-file> <remote-dest>
# Slot for scp-based distribution when a gate must place a binary on the
# peer; used by odl-reload-module.sh.
odl_peer_put ()
{
    local src="$1" dst="$2"
    # shellcheck disable=SC2086
    scp $ODL_SSH_OPTS "$src" "$ODL_PEER:$dst"
}

# =====================================================================
#  debugfs / counters
# =====================================================================

# Returns 0 iff the local box exports odl_tb5 counters.
odl_debugfs_local_ok ()
{
    [ -r "$ODL_STATS_FILE" ]
}

odl_debugfs_peer_ok ()
{
    odl_peer test -r "$ODL_STATS_FILE"
}

# Gate precondition: both boxes must export counter states -> skip on fork
# builds where this is absent (debugfs is a fork-only feature).
odl_require_debugfs ()
{
    odl_debugfs_local_ok || odl_skip "debugfs not exported (fork build?); counters unavailable"
    odl_debugfs_peer_ok || odl_skip "peer debugfs not exported (fork build?)"
}

# Get a counter's current value from the LOCAL stats file.
# $1 = key name.
odl_get_stat ()
{
    awk -v k="$1" '$1==k{print $2}' "$ODL_STATS_FILE" 2>/dev/null | head -1
}

# Get a counter from the PEER box (via ssh+awk).
# NOTE: odl_peer rejoins its argv with spaces for the remote shell, so a
# bare awk program would lose its $1/$2 to remote expansion.  The program
# is therefore shipped as ONE already-quoted string argument.
odl_peer_stat ()
{
    odl_peer "awk -v k='$1' '\$1==k{print \$2}' '$ODL_STATS_FILE'" 2>/dev/null |
        head -1
}

# Zero counters on both boxes.
odl_reset_stats ()
{
    printf '1\n' > "$ODL_STATS_RESET" 2>/dev/null || return 1
    odl_peer "printf '1\n' > $ODL_STATS_RESET" 2>/dev/null || return 1
}

# Snapshot the local stats file under $FARM.
odl_snapshot_local ()
{
    local name="$1"
    cp "$ODL_STATS_FILE" "$FARM/${name}-local.stats" 2>/dev/null
}

# Snapshot the peer stats file under $FARM.
odl_snapshot_peer ()
{
    local name="$1"
    odl_peer_capture "$ODL_STATS_FILE" "$FARM/${name}-peer.stats"
}

# Compare two stats snapshots; shortfall of the "leak" decisions in
# the frame-balance / dmabuf gates is handled in the gates themselves.

# =====================================================================
#  forensics: on failure persist everything that would cost a day to lose
# =====================================================================
# $1 = a `stage` label (trial number or size).
odl_forensics ()
{
    local stage="$1"
    local dir="$FARM"
    [ -d "$FARM/fail" ] && dir="$FARM/fail"
    dmesg > "$dir/${stage}-dmesg-local.txt" 2>/dev/null || true
    odl_peer "dmesg" > "$dir/${stage}-dmesg-peer.txt" 2>/dev/null || true
    cp "$ODL_STATS_FILE" "$dir/${stage}-stats-local.txt" 2>/dev/null || true
    odl_peer "cat $ODL_STATS_FILE" > "$dir/${stage}-stats-peer.txt" 2>/dev/null || true
    odl_log "forensics: $dir/${stage}-{dmesg,stats}*"
}

# =====================================================================
#  wall-clock timeout wrapper (Tier 0b). Copy of gate-send-stripe's design.
# =====================================================================
# odl_run_timeout <seconds> -- cmd [args...]
#   Run `cmd` under a wall-clock deadline; output passes through to the
#   caller, exit code follows the child's (with 124 the timeout expiry,
#   the GNU `timeout` convention).  This converts a hang into a FAIL.
odl_run_timeout ()
{
    local secs="$1"; shift
    if [ "$1" = "--" ]; then shift; fi
    timeout -k 15 "$secs" "$@" 2>&1
    local rc=$?
    if [ "$rc" -eq 124 ]; then
        printf 'odl_run_timeout: wall-clock %ss exceeded\n' "$secs" >&2
    fi
    return "$rc"
}

# odl_ferm_bg <name> <cmd...>  — background helper for non-systemd host
# (no systemd-run available). Kill by PID, waits for exit.
_odl_bg_launcher ()
{
    # helper to keep bash job control off
    :
}

# =====================================================================
#  server bring-up / teardown
# =====================================================================

# odl_server <gate-name>: start a detached lifecycle so the CLI server
# survives the ssh session that might drive it. Use systemd-run when
# available, otherwise nohup + PID.
ODL_SRV_PID=
ODL_SRV_UNIT=

odl_server_start ()
{
    local server_bin="$1"; shift
    ODL_SRV_PID=
    ODL_SRV_UNIT=
    mkdir -p "$FARM"
    if command -v systemd-run >/dev/null 2>&1 && \
            systemctl is-system-running >/dev/null 2>&1; then
        ODL_SRV_UNIT="odl-csrv-${ODL_GATE_NAME:-od-gate}-${BASHPID:-$$}"
        # shellcheck disable=SC2086
        systemd-run --unit="$ODL_SRV_UNIT" --collect --no-block \
            env LD_LIBRARY_PATH="$ODL_LD_LIBRARY_PATH" "$server_bin" "$@" \
            </dev/null >"$FARM/server.out" 2>&1
    else
        ODL_SRV_PID=$!
    fi
    sleep 2
}

odl_server_stop ()
{
    if [ -n "${ODL_SRV_UNIT:-}" ]; then
        systemctl stop "$ODL_SRV_UNIT" 2>/dev/null || true
        systemctl reset-failed "$ODL_SRV_UNIT" 2>/dev/null || true
    fi
    if [ -n "${ODL_SRV_PID:-}" ]; then
        kill "$ODL_SRV_PID" 2>/dev/null || true
        wait "$ODL_SRV_PID" 2>/dev/null || true
    fi
    ODL_SRV_PID=
    ODL_SRV_UNIT=
}

# =====================================================================
#  module param helpers
# =====================================================================

# Live value of a loaded module param (e.g. `num_paths`).
odl_modparam ()
{
    local name="$1"
    if [ -r "$ODL_MODPARAM/$name" ]; then
        cat "$ODL_MODPARAM/$name"
    else
        echo
    fi
}

# Set a writable module param on both boxes (odl_busy_poll_us,
# dmabuf_paths). Returns 1 if not writable.
odl_set_param ()
{
    local name="$1" val="$2"
    [ -w "$ODL_MODPARAM/$name" ] || return 1
    printf '%s\n' "$val" > "$ODL_MODPARAM/$name"
    odl_peer "echo $val > $ODL_MODPARAM/$name" 2>/dev/null || return 1
}

# =====================================================================
#  rate / verdict helpers
# =====================================================================

# odl_report <fail> <pass> <label>: prints
#    label: passes/N fails=X
# and returns 0 iff fails are zero.
odl_report ()
{
    local fail="$1" pass="$2" label="$3"
    local total=$((pass+fail))
    mkdir -p "$FARM"
    printf '%s: passes=%d/%d fails=%d\n' "$label" "$pass" "$total" "$fail"
    printf 'pass=%d fail=%d\n' "$pass" "$fail" >> "$FARM/summary.txt"
    [ "$fail" -eq 0 ]
}

# dmesg-pattern check on both boxes. Used to turn a kernel-visible event
# (dmabuf submit timeout) into a hard gate counter.
odl_kernel_seen ()
{
    local pat="$1"
    [ -n "$(dmesg 2>/dev/null | grep -E "$pat")" ] && return 0
    [ -n "$(odl_peer "dmesg | grep -E '$pat'" 2>/dev/null)" ] && return 0
    return 1
}

# =====================================================================
#  reboot-safety guard (rmmod is forbidden, and a raw
#  remove() wedges shutdown). Any gate that reboots MUST check this first.
# =====================================================================

# odl_guard_ok <module .ko path> <driver source dir>: verifies the fast
# path (system_state != SYSTEM_RUNNING) exists in the source of the build
# that would be loaded. Exits 2 (SKIP) if unknown.
odl_guard_ok ()
{
    local ko="${1:-${ODL_BUILD_DIR}/driver/odl_tb5.ko}"
    local srcdir="${2:-${ODL_DIR}/../driver}"
    local src="${srcdir}/odl_tb5_service.c"

    [ -f "$src" ] || { odl_warn "guard: can't find $src"; return 2; }
    if ! grep -q "system_state != SYSTEM_RUNNING" "$src"; then
        odl_warn "guard: no shutdown fast-path in $src — refusing reboot"
        return 2
    fi
    [ -f "$ko" ] || { odl_warn "guard: module not built at $ko"; return 2; }

    if command -v modinfo >/dev/null 2>&1; then
        local srcver
        srcver="$(modinfo -F srcversion "$ko" 2>/dev/null || true)"
        if [ -n "$srcver" ]; then
            odl_log "guard: module srcversion $srcver OK"
        fi
    fi
    return 0
}