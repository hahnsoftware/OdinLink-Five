#!/usr/bin/env bash
#
# odl-gate-preflight — Tier 2a: the cheapest gate in the suite, and the
# only one that runs ANYWHERE (no rig, no cable) — the loopback datapath
# makes it CI-ready.
#
# Two checks, in order:
#
#   1. Strict userspace compile  (-Wall -Wextra -Werror) of the lib and
#      the CLI.  Catches the trivial regressions without touching the box.
#      (Kernel-module build vs the running headers is optional, OFF by
#      default — set ODL_BUILD_MODULE=1 and a usable build dir.)
#
#   2. Loopback datapath smoke (any Linux box, no cable):
#        - module not loaded  -> insmod with loopback=1, run tests, rmmod
#          (safe here — no xdomain exists, so rmmod cannot deadlock)
#        - module loaded with loopback=N, N>0  -> run tests directly
#        - module loaded without loopback (the real rig / real device)
#          -> SKIP this part: reload is forbidden, and the loopback/nonloop
#          split is deliberate
#
#      Runs:  $ODL_TEST_BIN  (odl_tb5_test) — the repo lifecycle/device
#      suites — against /dev/odl_tb5_0.
#
#   3. No rig-safety involvement: nothing reboots, nothing loads on the
#      test rig.
#
# Caveat on scope: loopback covers the ioctl API surface (streams,
# legacy, mmap, poll) and the lifecycle — NOT payload integrity, because
# loopback has no stream send/recv hooks wired (no rings).  Payload
# verification needs a real link; that is odl-gate-integrity's job.
#
# Usage:
#   odl-gate-preflight.sh [--no-compile] [--no-loopback] [--build-module]
# Env: ODL_BUILD_DIR, ODL_TEST_BIN, ODL_MODULE_KO, ODL_WERROR=0 to relax
#
# Exit: 0 pass, 1 fail, 2 skip (module-loaded-without-loopback case only
#       for the loopback part; compile failures are hard failures).

set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$DIR/od/common.sh"

ODL_MODULE_KO="${ODL_MODULE_KO:-${ODL_DIR}/driver/odl_tb5.ko}"
ODL_SRC_LIB="${ODL_SRC_LIB:-${ODL_DIR}/lib}"
ODL_SRC_CLI="${ODL_SRC_CLI:-${ODL_DIR}/cli}"
ODL_SRC_UAPI="${ODL_SRC_UAPI:-${ODL_DIR}/driver/uapi}"
ODL_SRC_RCCL="${ODL_SRC_RCCL:-${ODL_DIR}/third_party/rccl}"

DO_COMPILE=1
DO_LOOPBACK=1
DO_BUILD_MODULE=0
WERROR="${ODL_WERROR:-1}"

while [ $# -gt 0 ]; do
    case "$1" in
        --no-compile)     DO_COMPILE=0; shift ;;
        --no-loopback)    DO_LOOPBACK=0; shift ;;
        --build-module)   DO_BUILD_MODULE=1; shift ;;
        -h|--help) sed -n '2,40p' "$0"; exit 0 ;;
        *) odl_die "unknown arg: $1" ;;
    esac
done

odl_setup "gate-preflight"

fails=0

# ---------------------------------------------------------------- 1. compile
if [ "$DO_COMPILE" = "1" ]; then
    odl_log "step 1/3: strict userspace compile (-Wall -Wextra [-Werror])"

    werror=""; [ "$WERROR" = "1" ] && werror="-Werror"
    # shellcheck disable=SC2086
    ok=1
    for src in "$ODL_SRC_LIB"/src/*.c "$ODL_SRC_CLI"/src/*.c; do
        if ! gcc -fsyntax-only -std=gnu11 -Wall -Wextra $werror \
                -I "$ODL_SRC_LIB/include" -I "$ODL_SRC_UAPI" \
                -I "$ODL_SRC_RCCL" -I "$ODL_SRC_CLI/src" "$src" 2>"$FARM/cc-$(
                    basename "$src").err"; then
            odl_log "  compile FAIL: $src (see farm cc-*.err)"
            fails=$((fails+1))
        fi
    done
    if [ "$fails" -eq 0 ]; then
        odl_log "  compile: PASS"
    else
        odl_log "  compile: FAIL ($fails file(s))"
    fi

    if [ "$DO_BUILD_MODULE" = "1" ]; then
        odl_log "  optional: build kernel module (headers must match host)"
        if [ -d "$ODL_BUILD_DIR" ]; then
            if cmake --build "$ODL_BUILD_DIR" --target driver 2>>"$FARM/module-build.log"; then
                odl_log "  module: build OK"
            else
                odl_log "  module: build FAIL (see module-build.log)"
                fails=$((fails+1))
            fi
        else
            odl_skip "build dir $ODL_BUILD_DIR missing — module build needs it; compile-only mode"
        fi
    fi
fi

# ----------------------------------------------------------- 2. loopback
if [ "$DO_LOOPBACK" = "1" ]; then
    odl_log "== 2: loopback datapath smoke"
    lb="$(odl_modparam loopback)"
    loaded="$(lsmod 2>/dev/null | grep -c '^odl_tb5')"; [ "${loaded:-0}" != "0" ] && loaded="1" || loaded=""

    if [ "$loaded" = "0" ]; then
        if [ ! -f "$ODL_MODULE_KO" ]; then
            odl_skip "module not loaded and $ODL_MODULE_KO missing"
        fi
        odl_log "  module not loaded — insmod loopback=1 (safe: no xdomain)"
        if ! sudo -n true 2>/dev/null; then
            odl_die "need passwordless sudo to insmod for loopback smoke"
        fi
        sudo -n insmod "$ODL_MODULE_KO" loopback=1 || odl_die "insmod failed"
        sleep 1
        DEV_LOADED=1
    elif [ "${loaded}" = "1" ] && [ "${lb:-0}" -gt 0 ]; then
        DEV_LOADED=1
        odl_log "module loaded with loopback=${lb} — using it"
    else
        odl_log "module loaded WITHOUT loopback (real device rig?) — SKIP loopback smoke"
        odl_log "  rmmod/insmod forbidden; load loopback=N on a dev box for this part"
        exit 2
    fi

    DEVNODE="/dev/odl_tb5_0"
    for i in $(seq 1 10); do
        [ -e "$DEVNODE" ] && break
        sleep 1
    done
    [ -e "$DEVNODE" ] || { odl_log "loopback node $DEVNODE never appeared"; fails=$((fails+1)); }

    if [ -x "$ODL_TEST_BIN" ]; then
        sudo -n chmod 666 "$DEVNODE" 2>/dev/null || true

        # shellcheck disable=SC2086
        "$ODL_TEST_BIN" >"$FARM/loopback-test.out" 2>&1
        rc=$?
        if [ "$rc" -eq 0 ]; then
            odl_log "  odl_tb5_test on loopback: PASS"
        else
            odl_log "  odl_tb5_test on loopback: FAIL (rc=$rc) — see farm/loopback-test.out"
            fails=$((fails+1))
        fi
    else
        odl_skip "test binary missing: $ODL_TEST_BIN"
    fi

    # tear down cleanly ONLY if we insmodded it ourselves
    if [ "${DEV_LOADED:-0}" = "1" ] && [ "$loaded" = "0" ]; then
        sudo -n rmmod odl_tb5 2>/dev/null || true
    fi
fi

# ----------------------------------------------------------------- summary
if [ "$fails" -eq 0 ]; then
    odl_log "gate-preflight: PASS"
    exit 0
else
    odl_log "gate-preflight: FAIL ($fails)"
    exit 1
fi