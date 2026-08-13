# Zero-Copy DMA-BUF Transfers — Developer Guide

This covers the internals of the DMA-BUF (zero-copy GPU) data path. For the
user/operator view, see `docs/zero-copy-dmabuf-user.md`.

## Two I/O models

OdinLink exposes two APIs that both sit on top of the same Thunderbolt ring DMA:

1. **Legacy double-buffer** (`odl_tb5_send/recv`, `odl_tb5_send_dmabuf/
   recv_dmabuf`) — the original framed model. `send_dmabuf` moves a DMA-BUF
   region instead of the host bounce buffer.
2. **Stream multiplexed I/O** (`odl_tb5_stream_*`) — kernel manages many
   logical streams over the same link; `stream_send_dmabuf` /
   `stream_recv_dmabuf` are the zero-copy variants.

Both are plumbed to the RCCL/NCCL plugins, which register GPU memory via the
plugin `regMrDmaBuf` entry point and then call the stream/DMA-BUF send/recv.

## DMA-BUF registration path

The plugin's `regMrDmaBuf` refuses unsupported registrations (the single-box
`odl_tb5_test` RCCL suite asserts this). Once a DMA-BUF fd is handed in, the
library passes it straight to the kernel via the stream ioctls:

```
ODL_TB5_IOCTL_STREAM_SEND_DMABUF  0x26   struct odl_tb5_stream_dmabuf
ODL_TB5_IOCTL_STREAM_RECV_DMABUF  0x27   struct odl_tb5_stream_dmabuf
```

(`driver/uapi/odl_tb5_uapi.h`). The kernel reads `dmabuf_fd`, `offset`, `len`,
and `stream_id` and drives the DMA engine at the buffer's backing pages — no
host copy.

## The raw zero-copy path (`raw_payload_ok`)

When both peers negotiate raw-payload capability (v3 login, `F_RAW_PAYLOAD`,
only when `odl_protocol_mode == 0`), the device sets `dev->raw_payload_ok`
(`driver/odl_tb5_core.h:428`, assigned throughout `driver/odl_tb5_proto.c`).
When set, the TX submit path:

- writes **no stream header** and does **no memcpy**;
- points the frame directly at the buffer: `frame->buffer_phy =
  sg_dma_address(sg) + seg_off`;
- sets `frame->size = chunk` (TX) and advertises `size = 0` on RX (the link
  counts 4096-byte slots, so a `chunk`-byte payload is absorbed);
- marks the frame end with `frame->eof = ODL_TB5_PDF_EOF_DATA`
  (`driver/odl_tb5_ring_dma.c:2067`).

RX for a raw frame posts per eligible cell at the exporter page. The advantage
over the framed path is eliminating the copy through the host double buffer.

### EOF markers — corrected

The uAPI defines `ODL_TB5_PDF_EOF_DATA 0x02` (`driver/uapi/odl_tb5_uapi.h:58`)
and `ODL_TB5_PDF_EOF_CTRL` for control frames. **Raw frames carry
`EOF_DATA`, not a separate raw EOF marker.** An earlier handoff note claimed a
dedicated `ODL_TB5_PDF_EOF_RAW` marker was (re)added; that is incorrect — no
such symbol is defined or used anywhere in the tree (every `frame->eof`
assignment in `ring_dma.c` uses `EOF_DATA`/`EOF_CTRL`). Raw-ness is a
*negotiated* property (`raw_payload_ok`), never a wire marker.

## Diagnostics & counters

Raw-path observability (defined in `driver/odl_tb5_core.h`, incremented in
`ring_dma.c`):

| Counter | Meaning | Site |
|---------|---------|------|
| `raw_tx_frames` | raw TX frames submitted | `ring_dma.c:2080` |
| `raw_rx_len_mismatch` | RX length did not match expected | `ring_dma.c:2198` |
| `raw_eligible_reject` | eligible raw cell rejected (capability dropped) | `ring_dma.c:1854`, `:1883` |
| `raw_unaligned_fallback` | unaligned request fell back to framed | `ring_dma.c:1837` |

Loss detection is **message-level**, not per-frame `frag_idx`: a
sum/count mismatch drops the whole transfer and bumps a counter.

General RX observability added alongside this work:

- `rx_unclassified` — frames that completed with a valid size but no DMA magic
  and no recognized stream header (control frames arriving in an unexpected
  layout, not lost frames). Counted and warned (first 3) in
  `ring_dma.c` so the exact contents are on record.
- `restart_count` — reconnect-loop tally, one increment per verify-failure
  restart.

These are exposed via the device's debugfs/stats; `chardev.c` already prints
`cur_raw_payload_ok`.

## Test layout

| Binary | Suites / scope | Needs peer? |
|--------|----------------|------------|
| `odl_tb5_test` | device, lib_api, plugin (3 suites) | no — single box |
| `odl_tb5_test_rccl_dmabuf` | DMA-BUF fd-plumbing / registration | no (transfer times out single-box by design) |
| `odl_tb5_pair_dmabuf` | legacy + stream DMA-BUF e2e, pattern-checked | **yes, two boxes** |
| `odl_stream_verify` | N streams × M rounds, content-verified | **yes, two boxes** |

### Single-box starvation (by design)

Synchronous DMA-BUF traffic uses the reserved data rings (1..`num_paths-1`).
A TX frame only completes once the **peer** has posted RX staging frames. One
box alone starves its own sends — this is intentional, not a bug. The paired
test therefore posts the receiver's RX staging **first** (an RCCL/NCCL-style
rendezvous: recv is armed before send) and retries up to ~60 s while the sender
is started. `odl_tb5_test_rccl_dmabuf` returning `-110` on a lone box is the
expected manifestation of this.

### Test-harness bug fixed alongside this feature

`odl_tb5_pair_dmabuf` previously crashed at cleanup (`odl_tb5_stream_close` /
`odl_tb5_close`) under `-O2`. Root cause: `pthread_join(tid, (void **)&handled)`
wrote an 8-byte `void *` into a 4-byte `int handled`, overflowing the adjacent
`handle` and corrupting it. Fixed by joining into a `void *` and casting.

## Known issue — width-1 fallback stalls on the first transfer after stream load

**Affects only `dmabuf_paths=1`** (or a link that negotiates a single path).
The default (`dmabuf_paths=0`, i.e. use all negotiated paths) is not affected;
see the measurements below.

With one path there is no dmabuf-reserved ring, so payload shares the control
ring with the frame pool. `odl_tb5_submit_dmabuf` raises `dmabuf_rx_active`,
which stops *new* pool frames from queueing, and drops `rx_target` to 0 — but
pool frames already posted by `odl_tb5_rx_arm` still sit in the path-0 FIFO
**ahead** of the payload, and the NHI delivers inbound data into them. The
dmabuf frames behind never complete:

```
odl_tb5: dmabuf RX submit timeout on path 0
         (completed=0 submitted=261 tx_inflight=0 poll_active=1 e2e=1)
```

Reproducer (deterministic — 5 failures in 9 trials):

```bash
# arm the pool with stream traffic, then transfer at width 1
./tools/gates/odl-gate-sweep.sh
echo 1 | tee /sys/module/odl_tb5/parameters/dmabuf_paths   # both boxes
./tools/gates/odl-gate-stripe.sh --trials 3 --widths 1
```

Trial 1 fails, occasionally trial 2; later trials pass, because the failed
transfer leaves `rx_target` at 0 and the pool stays disarmed. Without the
stream-load step the rate drops to ~1 in 39. At the default width the same
sequence is clean (15/15).

A bounded drain wait before posting payload (`rx_quiesce_ms`, default 250 ms)
was tried and removed. It is not the fix, measured both ways with the same
trigger and the same userspace, varying only the module:

| build | width-1 failures |
|---|---|
| with the drain | 7 / 12 |
| without | 5 / 9 |

Indistinguishable. Both failure shapes were observed on the build that *has*
the drain: sometimes it expires — `rx_shared quiesce expired (2048 pool frames
still ahead of payload on path 0)` — and sometimes it drains the window
completely within its budget and the transfer stalls regardless. The second
case is the interesting one: emptying the pool window is not sufficient, so
retiring the frames is not the missing piece. The drain only added up to
250 ms to every width-1 submit.

Forcing the ring empty with
`tb_ring_stop`/`tb_ring_start` is **not** an option either — a restarted ring
resumes at an NHI descriptor position the driver no longer matches, which
produced measured 5/10 interleaved corruption. A real fix has to stop the pool
frames from being posted ahead of the payload in the first place, not try to
retire them afterwards.
