#!/usr/bin/env bash
#
# run-suite — orchestrator for the full OdinLink gate suite.
#
# Runs on the box that drives the rig; the second box is reached over SSH
# as $ODL_PEER.  Every gate is run to completion regardless of its result
# so one run produces a full picture; the per-gate exit codes are echoed
# (0=PASS 1=FAIL 2=SKIP).
#
# Configuration comes from tools/gates/rig.conf (see rig.conf.example) or
# from the environment; nothing here is hardcoded to a particular host.
#
# Usage: ./tools/gates/run-suite.sh
set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -f "$DIR/rig.conf" ] && . "$DIR/rig.conf"

: "${ODL_PEER:?set ODL_PEER (user@host of the second rig box) in rig.conf}"
export ODL_PEER
export ODL_BUILD_DIR="${ODL_BUILD_DIR:-$DIR/../../build}"
export ODL_FARM_PREFIX="${ODL_FARM_PREFIX:-/root/odl-gates}"
export ODL_TRIALS="${ODL_TRIALS:-20}"
export ODL_TRIAL_TIMEOUT="${ODL_TRIAL_TIMEOUT:-300}"
export ODL_STOP_ON_FAIL="${ODL_STOP_ON_FAIL:-1}"
export ODL_EXPECTED_PATHS="${ODL_EXPECTED_PATHS:-2}"
export ODL_LINK_SPEED_GBPS="${ODL_LINK_SPEED_GBPS:-0}"
export ODL_FORK_BUILD="${ODL_FORK_BUILD:-1}"

# Per-gate wall clock.  A wedged gate must not wedge the suite.
GATE_TIMEOUT="${ODL_GATE_TIMEOUT:-1800}"

chmod +x "$DIR"/*.sh "$DIR"/od/*.sh 2>/dev/null

run () {
  local name="$1"; shift
  echo "=================================================================="
  echo "[ORCH] START $name $(date +%H:%M:%S)"
  timeout "$GATE_TIMEOUT" bash "$DIR/$name.sh" "$@"
  local rc=$?
  echo "[ORCH] $name rc=$rc $(date +%H:%M:%S)  (0=PASS 1=FAIL 2=SKIP)"
  echo
  return 0
}

# Tier 0
run odl-link-check
# Tier 1
run odl-gate-dmabuf --trials 20
run odl-gate-integrity --trials 2 --streams 4 --rounds 8
# Tier 2
run odl-gate-preflight --no-loopback
run odl-gate-frame-balance --load bench
run odl-gate-latency --runs 20
# Tier 3
run odl-gate-stripe --trials 20
run odl-gate-sweep
# Tier 3: kernel residue
run odl-kernel-residue

echo "=================================================================="
echo "[ORCH] suite complete"
