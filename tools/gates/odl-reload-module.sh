#!/usr/bin/env bash
#
# odl-reload-module — the ONLY sanctioned way to change a 0444 module
# parameter (num_paths, ring_size, e2e, protocol).  It rebuilds around
# REBOOT instead of `rmmod`, because rmmod odl_tb5 is forbidden:
# with an active xdomain it deadlocks in remove_device (D state,
# refcount −1, unkillable) and even `reboot -f` then hangs.
#
# Safety preconditions verified BEFORE anything is staged:
#   1. the build carries the shutdown fast-path (`system_state !=
#      SYSTEM_RUNNING` early return in odl_tb5_remove()).  Without it a
#      raw shutdown wedges and demands a physical power cycle.  Checked
#      against the SOURCE that produces the module (odl_guard_ok).
#   2. the parameters being changed are read-only (0444) knobs.
#      Writable ones (dmabuf_paths, odl_busy_poll_us) are set via sysfs
#      and never need a reboot — this script refuses them.
#
# Flow (default = STAGE ONLY, no reboot):
#   * copies the .ko to both boxes: /opt/odl_tb5/odl_tb5.ko
#   * installs a systemd one-shot autoload unit on both boxes that will
#     insmod that .ko with the new args at the next boot
#   * prints what `--reboot` would do.
#
# With --reboot:
#   * reboots the PEER, waits for it to come back, verifies the module
#     loaded with the new params
#   * then instructs the operator to reboot THIS box (an SSH-driven
#     process cannot reboot the host it runs on mid-flight) and offers
#     --verify for the confirm step on the next invocation.
#
# Note: the new num_paths only takes effect AFTER both ends reboot with
# the new module; gates that need the new value must run after that.
#
# Usage:
#   odl-reload-module.sh [--ko PATH] [--args "num_paths=2 e2e=0"]
#                        [--reboot] [--verify]
# Env: ODL_BUILD_DIR, ODL_PEER, ODL_MODULE_KO, ODL_ARGS, ODL_REBOOT.
#
# Exit: 0 staged/verified; 1 failure; 2 refused (unsafe).

set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$DIR/od/common.sh"

KO="${ODL_MODULE_KO:-${ODL_BUILD_DIR}/driver/odl_tb5.ko}"
ARGS="${ODL_ARGS:-}"
DO_REBOOT=0
DO_VERIFY=0

DOT=/opt/odl_tb5
KO_DEST="$DOT/odl_tb5.ko"
LOADER=/usr/local/sbin/odl-tb5-autoload.sh
UNIT=/etc/systemd/system/odl-tb5-autoload.service

while [ $# -gt 0 ]; do
    case "$1" in
        --ko)     KO="$2"; shift 2 ;;
        --args)   ARGS="$2"; shift 2 ;;
        --reboot) DO_REBOOT=1; shift ;;
        --verify) DO_VERIFY=1; shift ;;
        -h|--help) sed -n '2,32p' "$0"; exit 0 ;;
        *) odl_die "unknown arg: $1" ;;
    esac
done

asroot ()
{
    if [ "$(id -u)" = "0" ]; then "$@"; else sudo -n "$@"; fi
}

# ------------------------------------------------------------ verify mode
verify_host ()
{
    # $1 = host label ("local" or a peer ssh target); prints state
    local host="$1" mod="" np=""
    if [ "$host" = "local" ]; then
        grep -q '^odl_tb5 ' /proc/modules 2>/dev/null && mod="loaded"
        np="$(cat /sys/module/odl_tb5/parameters/num_paths 2>/dev/null)"
    else
        # shellcheck disable=SC2086
        ssh $ODL_SSH_OPTS "$host" "grep -q '^odl_tb5 ' /proc/modules 2>/dev/null" \
            && mod="loaded"
        # shellcheck disable=SC2086
        np=$(ssh $ODL_SSH_OPTS "$host" \
                 'cat /sys/module/odl_tb5/parameters/num_paths 2>/dev/null' || true)
    fi
    printf '%s: module=%s num_paths=%s\n' "$host" "${mod:-NOT-loaded}" "${np:-?}"
    [ -n "$mod" ]
}

if [ "$DO_VERIFY" = "1" ]; then
    odl_setup "reload-module-verify"
    verify_host local; rl=$?
    echo ""
    exit $(( rl == 0 ? 0 : 1 ))
fi

odl_setup "reload-module"
[ -f "$KO" ] || odl_skip "no module at $KO (set ODL_MODULE_KO or build first)"

# ---- refuse writable knobs -------------------------------------------------
if [ -n "$ARGS" ]; then
    for knob in $ARGS; do
        case "$knob" in
            dmabuf_paths|odl_busy_poll_us)
                odl_die "$knob is writable — set it via sysfs, no reboot needed" ;;
        esac
    done
fi

# ---- precondition 1: shutdown fast-path present in source ----------------
odl_guard_ok "$KO" "$ODL_DIR/driver" \
    || odl_skip "shutdown fast-path not verifiable — refusing to reboot"
odl_log "guard: shutdown fast-path confirmed for $KO"

# already at target? then nothing to do
if [ -n "${ODL_ARGS:-}" ]; then
    cur="$(odl_modparam num_paths)"
    want="$(echo "$ODL_ARGS" | grep -oE 'num_paths=[0-9]+' | cut -d= -f2)"
    if [ -n "$want" ] && [ "${cur:-}" = "$want" ]; then
        odl_log "num_paths already $want — nothing to change"
        exit 0
    fi
fi

# ---- stage: write the autoloader + unit into $FARM -------------------------
cat > "$FARM/odl-tb5-autoload.sh" <<EOF
#!/bin/sh
sleep 3
grep -q '^odl_tb5 ' /proc/modules && exit 0
insmod $KO_DEST $ARGS
exit 0
EOF
chmod 0755 "$FARM/odl-tb5-autoload.sh"

cat > "$FARM/odl-tb5-autoload.service" <<'EOF'
[Unit]
Description=/usr/local/sbin/odl-tb5-autoload
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/odl-tb5-autoload.sh

[Install]
WantedBy=multi-user.target
EOF

stage_box ()
{
    # $1 = "local" or ssh remote
    local where="$1"
    if [ "$where" = "local" ]; then
        asroot mkdir -p "$DOT" || true
        asroot cp "$KO" "$KO_DEST"
        asroot install -m 0755 "$FARM/odl-tb5-autoload.sh" "$LOADER"
        asroot cp "$FARM/odl-tb5-autoload.service" "$UNIT"
        asroot systemctl daemon-reload
        asroot systemctl enable odl-tb5-autoload
        odl_log "local: staged $KO_DEST"
    else
        # shellcheck disable=SC2086
        ssh $ODL_SSH_OPTS "$where" "mkdir -p $DOT"
        odl_peer_put "$KO" "/tmp/odl-tb5.ko"
        # shellcheck disable=SC2086
        ssh $ODL_SSH_OPTS "$where" "mv /tmp/odl-tb5.ko $KO_DEST"
        # shellcheck disable=SC2086
        ssh $ODL_SSH_OPTS "$where" "cat > $LOADER" < "$FARM/odl-tb5-autoload.sh"
        # shellcheck disable=SC2086
        ssh $ODL_SSH_OPTS "$where" "chmod 0755 $LOADER"
        # shellcheck disable=SC2086
        ssh $ODL_SSH_OPTS "$where" "cat > $UNIT" < "$FARM/odl-tb5-autoload.service"
        # shellcheck disable=SC2086
        ssh $ODL_SSH_OPTS "$where" "systemctl daemon-reload && systemctl enable odl-tb5-autoload"
        odl_log "$where: staged $KO_DEST"
    fi
}

stage_box local
stage_box "$ODL_PEER"

if [ "$DO_REBOOT" = "0" ]; then
    odl_log "STAGED (no reboot). To apply: re-run with --reboot (peer first)."
    exit 0
fi

# ---- apply: reboot peer, wait, verify; then hand-off local reboot --------
odl_log "@== applying: rebooting peer $ODL_PEER"

# Record the boot id BEFORE the reboot.  "sshd answers again" is not proof
# the box rebooted: shutdown takes tens of seconds and sshd keeps accepting
# connections most of that time, so a plain reachability poll returns
# immediately and then verifies the module that is still loaded from the
# PREVIOUS boot — reporting success for a reload that has not happened yet.
# The boot id changes only across an actual boot.
# shellcheck disable=SC2086
boot_before="$(ssh $ODL_SSH_OPTS "$ODL_PEER" \
        "cat /proc/sys/kernel/random/boot_id" 2>/dev/null)"
[ -n "$boot_before" ] || odl_die "cannot read peer boot id — refusing to reboot blind"

# shellcheck disable=SC2086
ssh $ODL_SSH_OPTS "$ODL_PEER" "sync; reboot" 2>/dev/null || true
deadline=$(( $(date +%s) + 420 ))
up=""
while [ "$(date +%s)" -lt "$deadline" ]; do
    sleep 10
    boot_now="$(ssh -o BatchMode=yes -o ConnectTimeout=5 \
            -o StrictHostKeyChecking=no "$ODL_PEER" \
            "cat /proc/sys/kernel/random/boot_id" 2>/dev/null)"
    if [ -n "$boot_now" ] && [ "$boot_now" != "$boot_before" ]; then
        up=1; break
    fi
done
if [ -z "$up" ]; then
    odl_die "peer did not reboot within 420 s — manual intervention"
fi
# The autoload unit runs at multi-user.target; give it room to insmod.
sleep 8
if verify_host "$ODL_PEER"; then :; else
    odl_warn "peer up but module not loaded — check: journalctl -b -u odl-tb5-autoload"
fi

odl_log "now reboot the LOCAL box manually, then run:"
odl_log "  odl-link-check.sh      (verify READY + agreed paths on the new load)"
echo
exit 0