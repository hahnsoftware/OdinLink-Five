#!/usr/bin/env bash
#
# odl-run-timeout — Tier 0b: run any command under a wall-clock deadline
# so a hang is a FAIL, never a hung gate.
#
# The design is lifted wholesale from the thunderbolt-ibverbs
# `gate-send-stripe` script, whose wall-clock discipline OdinLink lacked:
# today's dmabuf hang blocked forever with no timeout anywhere in the
# stack, and every occurrence had to be killed by PID from the outside.
#
# This wrapper is the single cheapest transferable idea in the whole
# original suite, and it is used by every gate in this directory (see
# odl_run_timeout in od/common.sh).  This module is just the stand-alone
# CLI form of the same thing.
#
# Usage:
#   odl-run-timeout.sh [-k grace] [-q] SECONDS -- command [args...]
#   -k <secs>   SIGKILL grace after TERM (default 15)
#   -q          quiet: only the exit code, no banners
# Env:
#   ODL_PIDFILE   write the child PID here (kill-by-PID hygiene)
#
# Exit: child's exit code; 124 if the deadline expired (TERM sent), 137
#   if the SIGKILL grace also expired.

set -u

KILLAFTER=15
QUIET=0

while getopts ":k:q" opt; do
    case "$opt" in
        k) KILLAFTER="$OPTARG" ;;
        q) QUIET=1 ;;
        *) echo "usage: $0 [-k grace] [-q] SECONDS -- cmd [args...]"; exit 2 ;;
    esac
done
shift $((OPTIND - 1))

if [ $# -lt 2 ]; then
    echo "usage: $0 [-k grace] [-q] SECONDS -- cmd [args...]" >&2
    exit 2
fi
DEADLINE="$1"; shift
[ "$1" = "--" ] && shift

if [ "$QUIET" -eq 0 ]; then
    echo "[odl-run-timeout] deadline ${DEADLINE}s on: $*"
fi

rc=0
if [ -n "${ODL_PIDFILE:-}" ]; then
    timeout -k "$KILLAFTER" "$DEADLINE" "$@" &
    bg=$!
    echo "$bg" > "$ODL_PIDFILE"
    wait "$bg"
    rc=$?
    rm -f "$ODL_PIDFILE"
else
    timeout -k "$KILLAFTER" "$DEADLINE" "$@"
    rc=$?
fi

if [ "$rc" -eq 124 ] && [ "$QUIET" -eq 0 ]; then
    echo "[odl-run-timeout] TIMEOUT after ${DEADLINE}s — command killed" >&2
fi
exit "$rc"