# OdinLink regression gates

A minimal, portable OdinLink regression suite, ported from the
`thunderbolt-ibverbs` gate suite.

Two lessons from the original suite shape every design choice here:

1. **Performance gates are blind to correctness.** Every one of
   `odl_tb5_cli`'s tests measures timing and byte counts; none compares a
   payload byte. A build whose dmabuf path can't complete a single
   transfer passes the whole CLI matrix at full speed. The one tool that
   sees it is the one that checks bytes — hence `odl-gate-integrity` and
   the new `tests/odl_stream_verify` tool.

2. **An intermittent failure needs a gate that reports a *rate*, not a
   verdict.** The dmabuf hang (root-caused and fixed on
   `experimental/upstream-sync`) passed 16/16 retests, then hung
   deterministically when the *trigger* (ring-full + next transfer) was
   finally swept. Every liveness-sensitive gate here reports **passes/N**
   and sweeps the trigger, not just a repeated size.

## Exit-code convention

| code | meaning |
|------|---------|
| 0    | PASS |
| 1    | FAIL |
| 2    | SKIP — missing prerequisite (peer, debugfs, parameter, binary) |

On OdinLink the polite-by-default skip is important: debugfs counter
exports exist only on the fork build, so counter gates *must* SKIP on
`origin/main` rather than FAIL.

## Rig model

One box runs the gate script; a second box ("peer", `ODL_PEER`) is
driven over SSH. Key rigidity:

- `rmmod odl_tb5` is **forbidden** (deadlocks with an active xdomain) —
  gates never reload the module. They run against the currently loaded
  module and SKIP when a required knob doesn't match (`gate-stripe` with
  `num_paths < 2`). The only way to change a 0444 param is the reboot
  path in `odl-reload-module.sh`.
- The `odl_tb5_cli` server dies after each test block — every gate
  restarts it (a fresh `systemd-run` unit) before every client call.
- Kill-by-PID only, never `pkill -f` with a name that also appears in an
  SSH command line (that kills the controlling shell — verified, exit
  255).
- `odl_link_check` is dot-sourced into the rig gates so a mistrained
  link surfaces as SKIP/FAIL before a measurement, instead of a phantom
  regression.

## Files

| file | tier | what it protects | first commit in |
|------|-----------------|------------------|-----------------|
| `od/common.sh` | — | shared plumbing: ssh, debugfs, forensics, timeout, server, guard | A |
| `odl-link-check.sh` | Tier 0a | READY state, path-count agreement, zero pre-existing fault counters | A |
| `odl-run-timeout.sh` | Tier 0b | wall-clock wrapper — hang ⇒ FAIL | A |
| `odl-gate-dmabuf.sh` | Tier 1a | zero-copy dmabuf path (rate gate, trigger-swept) | B |
| `odl-gate-integrity.sh` + `tests/odl_stream_verify.c` | Tier 1b | stream payload integrity (the only byte check on the stream path) | C |
| `odl-gate-preflight.sh` | Tier 2a | strict compile + `loopback=1` no-cable smoke | D |
| `odl-gate-frame-balance.sh` | Tier 2b | per-path frame conservation, `rx_frames_no_stream` asymmetry | E |
| `odl-gate-latency.sh` | Tier 2c | tail (p99.9) + mode occupancy + poll mechanism | F |
| `odl-gate-stripe.sh` | Tier 3 (6 + 8) | dmabuf stripe-width sweep; path-scaling over dmabuf | G |
| `odl-gate-sweep.sh` | Tier 3 (7) | wide size × streams matrix; `--stop-on-fail` | G |
| `odl-reload-module.sh` | deploy note | the ONLY 0444-param change path, built around reboot + guard check | G |
| `rig.conf.example` | — | default rig configuration skeleton | A |
| `run-suite.sh` | — | orchestrator: runs every gate in order, one summary | G |

Deferred (not implemented): `bench-rccl-sweep`
(GPU nodes required), dual-cable multirail (hardware), the q=8 framing
(no QP analogue — frame-conservation kept in `gate-frame-balance`).

## Quick start

```bash
# one-time rig config
cp tools/gates/rig.conf.example tools/gates/rig.conf
# edit ODL_PEER, ODL_BUILD_DIR, ODL_EXPECTED_PATHS

./tools/gates/odl-link-check.sh                 # Tier 0: is the link usable?
./tools/gates/odl-gate-dmabuf.sh --trials 20   # Tier 1: rate gate, dmabuf
./tools/gates/odl-gate-integrity.sh --trials 2 # Tier 1: stream payload bits

# Tier 2+
./tools/gates/odl-gate-preflight.sh
./tools/gates/odl-gate-frame-balance.sh --load bench
./tools/gates/odl-gate-latency.sh --runs 30

# or everything, in order, with a per-gate wall clock
./tools/gates/run-suite.sh
```

`ODL_PEER` is mandatory for the two-box gates; without it `odl_setup`
SKIPs. Every gate honours the same env block (see rig.conf.example).
Results (the "farm") land under `/root/odl-gates/<gate>-<timestamp>/`
(`ODL_FARM_PREFIX` to relocate).

## Rig safety

* No gate reboots by itself; only `odl-reload-module.sh` ever reboots,
  and it refuses unless the loaded build's source contains the shutdown
  fast-path (`system_state != SYSTEM_RUNNING` in `odl_tb5_remove()`).
* No counter-diff gate blanks `dmesg` before capturing first-failure
  forensics (dmesg + full stats from both boxes go to `*FARM*/fail/`).
* Oversized dmabuf requests returning `-ENOSPC` are the *correct* answer
  on a given ring (F2 capacity pre-check) — encoded as expected in the
  gate loops.
* `odl-gate-preflight` will `insmod loopback=1` on a box with no module
  loaded — and `rmmod`s only the instance it created (no xdomain exists
  then, so it cannot deadlock).

## Notes on this implementation's limits

* Executed against the two-box rig: `link-check`, `gate-dmabuf` (10/10),
  `gate-integrity` (2/2, 4 streams x 8 rounds), `gate-frame-balance`,
  `gate-latency` (30 runs), `gate-sweep` (8 cells) and
  `kernel-residue` all pass. `gate-stripe` and `reload-module` have not
  been run — both change a 0444 parameter, which needs the reboot path.
* `link-check` FAILs on a first run against a link that has been up for
  a while, because `rx_frames_canceled` is nonzero from an earlier ring
  teardown (2048 = one RX pool window). That is teardown residue, not a
  live fault. Reset the counters (`echo 1 > .../stats_reset` on both
  boxes) before the first gate of a session; the gate does not do this
  itself, deliberately, so it cannot hide a real pre-existing fault.
* The latency bimodality (~11 µs vs ~22 µs modes) is unexplained;
  `odl-gate-latency` reports mode occupancy first and gates second.
* `odl_tb5_bench_dmabuf` does not expose TX queue depth — `gate-stripe`
  sweeps depth indirectly via `--iters` and the trigger sizes, not via
  an explicit depth knob.

## Upstream grouping

The suite is split so each commit is independently upstreamable to
`origin/main` in a sensible order.

| group | contains | depends on |
|-------|----------|------------|
| A (base) | `od/common.sh`, `odl-link-check.sh`, `odl-run-timeout.sh`, `rig.conf.example` | — |
| B (dmabuf) | `odl-gate-dmabuf.sh` | A |
| C (integrity) | `tests/odl_stream_verify.c`, `tests/CMakeLists.txt`, `odl-gate-integrity.sh` | A |
| D (preflight) | `odl-gate-preflight.sh` | A |
| E (frame) | `odl-gate-frame-balance.sh` | A, B (intentionally trivial) |
| F (latency) | `odl-gate-latency.sh` | A |
| G (tail) | `odl-gate-stripe.sh`, `odl-gate-sweep.sh`, `odl-reload-module.sh` | A, B |

Each group is a self-contained feature; earlier groups do not depend on
later ones. Group E and F can land in either order.