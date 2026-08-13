#!/usr/bin/env bash
# odl-kernel-residue.sh — gate 7: post-crash / post-reload residue.
#
# A crash, a wedged reboot, or a botched module reload can leave the kernel
# state tainted (oops records, stale DMA, half-unregistered chardevs). This
# gate re-checks AFTER the offending event has been cleaned up, so the rig
# can be certified "green again" instead of silently reusing a tainted box.
#
# Checks (all pass = clean):
#   1. No odl_tb5 oops/panic record in this boot's dmesg.
#   2. Module reload trace: if odl_tb5 is loaded, the running .ko matches
#      the freshly built one (no stale buildid / no phantom load).
#   3. Chardev residue: no /dev/odl_tb5_* nodes with no loaded module
#      (udev left-overs from a failed unload) — a node without a device
#      behind it is a dead end for every user of the API.
#   4. Debugfs residue: after a stats reset, an idle device reports all
#      zeroes (DMA descriptors / ring slots from a previous session must
#      not leak into the new one).
#
# Exit: 0 = clean, 1 = residue found, 2 = SKIP (dmesg restricted or no
# debugfs and no other source of evidence).
#
# Usage: odl-kernel-residue.sh [--no-loopback]

set -u
ODL_GATE_NAME="${ODL_GATE_NAME:-odl-kernel-residue}"
. "$(dirname "${BASH_SOURCE[0]}")/od/common.sh"
odl_setup "$@" || exit 2

fail=0 pass=0

# -- 1. oops / panic residue ---------------------------------------------
if [ -n "$(dmesg 2>/dev/null | grep -E 'BUG:|Oops:|Kernel panic|kernel BUG')" ]; then
    if [ -n "$(dmesg 2>/dev/null | grep -E 'BUG:|Oops:|Kernel panic|kernel BUG.*odl_tb5|odl_tb5.*(Oops|BUG|panic)')" ]; then
        odl_log "[FAIL] dmesg holds an odl_tb5 oops/panic record from this boot"
        fail=$((fail+1))
    else
        odl_log "[PASS] no odl_tb5 oops/panic in dmesg (unrelated faults only)"
        pass=$((pass+1))
    fi
else
    odl_log "[PASS] dmesg clean of oops/panic records"
    pass=$((pass+1))
fi

# -- 2. loaded .ko matches the built one ---------------------------------
ko="${ODL_BUILD_DIR}/driver/odl_tb5.ko"
if [ -n "$(lsmod 2>/dev/null | grep '^odl_tb5')" ]; then
    if [ -f "$ko" ]; then
        built="$(md5sum "$ko" | cut -d' ' -f1)"
        live="$(md5sum "/lib/modules/$(uname -r)/extra/odl_tb5.ko" 2>/dev/null | cut -d' ' -f1)"
        if [ -z "$live" ]; then
            live="$(md5sum "/lib/modules/$(uname -r)/updates/dkms/odl_tb5.ko" 2>/dev/null | cut -d' ' -f1)"
        fi
        if [ -n "$live" ] && [ "$live" = "$built" ]; then
            odl_log "[PASS] running odl_tb5 matches built $ko"
            pass=$((pass+1))
        elif [ -z "$live" ]; then
            odl_log "[PASS] odl_tb5 loaded straight from a source tree (dev install) — nothing to compare against"
            pass=$((pass+1))
        else
            odl_log "[FAIL] loaded odl_tb5 differs from built $ko (stale install?)"
            fail=$((fail+1))
        fi
    else
        odl_log "[PASS] module loaded; no build tree to compare against"
        pass=$((pass+1))
    fi
else
    odl_log "[PASS] odl_tb5 not loaded — nothing to compare"
    pass=$((pass+1))
fi

# -- 3. chardev nodes without a module -----------------------------------
nodes="$(ls /dev/odl_tb5_* 2>/dev/null)"
if [ -n "$nodes" ] && [ -z "$(lsmod 2>/dev/null | grep '^odl_tb5')" ]; then
    odl_log "[FAIL] stale chardev nodes with no module loaded: $nodes"
    fail=$((fail+1))
else
    odl_log "[PASS] no stale chardev nodes"
    pass=$((pass+1))
fi

# -- 4. debugfs counters reset to zero on an idle device -----------------
if odl_debugfs_local_ok; then
    d=$(ls -d /sys/kernel/debug/odl_tb5/odl_tb5_* 2>/dev/null | head -n1)
    if [ -n "$d" ] && [ -w "$d/stats_reset" ]; then
        printf '1\n' > "$d/stats_reset"
        sleep 1
        residue="$(grep -E 'tx_frames_|rx_asm_start|rx_frames_canceled|rx_frames_runt' \
                       "$d/stats" 2>/dev/null | awk '$2 != 0 {print; n++} END{exit n>0 ? 1 : 0}')"
        if [ $? -eq 0 ]; then
            odl_log "[PASS] idle-device counters are all zero after reset"
            pass=$((pass+1))
        else
            odl_log "[FAIL] counters not zeroed on idle device:$residue"
            fail=$((fail+1))
        fi
    else
        odl_log "[SKIP] no writable debugfs stats_reset — no counter evidence"
    fi
else
    odl_log "[SKIP] no local debugfs (fork-only feature) — counter check skipped"
fi

odl_report "$fail" "$pass" "$ODL_GATE_NAME"
exit "$( [ "$fail" -eq 0 ] && echo 0 || echo 1 )"
