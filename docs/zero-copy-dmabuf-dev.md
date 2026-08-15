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

The plugin's `regMrDmaBuf` duplicates the dmabuf fd and keeps it (plus
the base offset) in the mhandle, which `isend`/`irecv` then pass to the
stream DMA-BUF ioctls — the single-box `odl_tb5_test` RCCL suite
asserts both the successful registration and the rejection of a bad
fd. Once a DMA-BUF fd is handed in, the library passes it straight to
the kernel via the stream ioctls:

```
ODL_TB5_IOCTL_STREAM_SEND_DMABUF  0x26   struct odl_tb5_stream_dmabuf
ODL_TB5_IOCTL_STREAM_RECV_DMABUF  0x27   struct odl_tb5_stream_dmabuf
```

(`driver/uapi/odl_tb5_uapi.h`). The kernel reads `dmabuf_fd`, `offset`, `len`,
and `stream_id` and drives the DMA engine at the buffer's backing pages — no
host copy.

## DMA-BUF ordering across connections

The kernel's dmabuf rings carry no stream header: the peer pairs a
posted RX transfer with a TX transfer purely by post order. RCCL runs
one proxy thread per channel, so transfers from different connections
can reach the plugin in any order — if the two boxes posted in
different orders, bytes would land in the wrong buffers. The plugin
makes the pairing deterministic with a device-wide control stream
(fixed id 250): the receiver announces each pending receive with a
READY message (sent atomically with the RX post under `g_wire_lock`),
and a single control-stream reader on the sender consumes READYs in
arrival order and posts the matching TX. READY order == RX post order
== TX post order on the wire by construction, so multi-channel RCCL
runs pair transfers identically on both boxes.

## The raw zero-copy path (`raw_payload_ok`)

When both peers negotiate raw-payload capability (v3 login, `F_RAW_PAYLOAD`,
only when `odl_protocol_mode == 0`), the device sets `dev->raw_payload_ok`
(`driver/odl_tb5_core.h:428`, assigned throughout `driver/odl_tb5_proto.c`).
When set, the TX submit path:

- writes **no stream header** and does **no memcpy**;
- points the frame directly at the buffer: `frame->buffer_phy =
  sg_dma_address(sg) + seg_off`;
- cuts the payload into `ODL_TB5_RAW_CELL_MAX` (2048) byte cells — a power of
  two dividing PAGE_SIZE, so every page-granular SG boundary from a DMA-heap
  or GPU exporter is also a cell boundary and both importers chunk the payload
  identically (see the geometry gate below);
- sets `frame->size = chunk` (TX) and advertises `size = 0` on RX (the link
  counts 4096-byte slots, so a `chunk`-byte payload is absorbed);
- marks the frame end with `frame->eof = ODL_TB5_PDF_EOF_DATA`
  (`driver/odl_tb5_ring_dma.c:2208`).

RX for a raw frame posts per eligible cell at the exporter page. The advantage
over the framed path is eliminating the copy through the host double buffer.

### The raw-geometry gate (`odl_tb5_dmabuf_walk`)

Raw eligibility is per-transfer and stricter than negotiation: a transfer is
raw only if **every cell except the transfer tail is a full cell**.  The raw
wire has no header, so the two importers must cut the payload at identical
boundaries; a short mid-transfer cell at an SG boundary on one host can be a
full cell at the same byte offset on the other (the 64-byte residue of a
4032-byte cell inside a 4096-byte page), which corrupts or stalls the receive.

The cell size is therefore pinned to `ODL_TB5_RAW_CELL_MAX = 2048`
(`driver/uapi/odl_tb5_uapi.h`), a power of two dividing PAGE_SIZE.  With
page-granular SG tables (every entry a multiple of PAGE_SIZE — the layout
`/dev/dma_heap/system`, amdgpu, and an IOMMU all produce), every SG boundary
coincides with a cell boundary, so non-tail cells are full by construction and
the chunking is deterministic in `len` alone.  A cell that does not divide
PAGE_SIZE makes the gate reject every multi-page buffer (`raw_eligible_reject`
rises and the transfer goes framed) and, because the hosts allocate
independently, lets one side stay raw while the other falls back — the
`raw_rx_len_mismatch` failure mode.  The rule is exercised by the single-box
regression test `tests/odl_tb5_raw_geom_test.c`.

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
| `raw_tx_frames` | raw TX frames submitted | `ring_dma.c:2289` |
| `raw_rx_len_mismatch` | RX length did not match expected | `ring_dma.c:1660` |
| `raw_eligible_reject` | eligible raw cell rejected (capability dropped) | `ring_dma.c:2061`, `:2090` |
| `raw_unaligned_fallback` | unaligned request fell back to framed | `ring_dma.c:2034` |

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
| `odl_tb5_raw_geom_test` | raw-geometry gate invariant (page-granular SG layouts) | no — single box, no module |
| `odl_tb5_pair_dmabuf` | legacy + stream DMA-BUF e2e, pattern-checked | **yes, two boxes** |
| `odl_stream_verify` | N streams × M rounds, content-verified | **yes, two boxes** |
| `odl_tb5_bench_dmabuf` | echo bench, `--allocator dmaheap\|amdgpu\|hip` | **yes, two boxes** |

## Allocators (`odl_tb5_bench_dmabuf --allocator`)

The DMA-heap path is the deterministic CPU backing the readiness gate's main
proof. Two GPU allocators export a **real amdgpu DMA-BUF** through the ROCm
driver stack, discovered at runtime via `dlopen` (the bench links no GPU
library and builds anywhere):

| Allocator | Backing | Requires |
|-----------|---------|----------|
| `dmaheap` (default) | `/dev/dma_heap/system` | `CONFIG_DMABUF_HEAPS_SYSTEM` |
| `amdgpu` | amdgpu BO via `libdrm_amdgpu` (`amdgpu_bo_alloc` + `amdgpu_bo_export` → dma-buf fd); **GEM domain follows GPU kind** — GTT on an iGPU/APU, VRAM on a dGPU | amdgpu loaded, `/dev/dri/cardN` (`/dev/amdgpu` is a udev alias, not required) |
| `hip` | the amdgpu export **imported into HIP** (`hipImportExternalMemory` + `hipExternalMemoryGetMappedBuffer`); fill/verify via `hipMemcpy` through the mapped pointer | amdgpu + a ROCm HIP runtime (`libamdhip64.so`) with ≥1 visible device |

When an allocator's prerequisites are missing the bench prints a SKIP reason
and exits 3. It **never silently falls back** to DMA-heap or memfd — a passed
run must be backed by the allocator the user asked for. The readiness gate
runs an optional second echo round (`--extra-allocator amdgpu|hip`, default
`hip`): a clean SKIP on **both** hosts is recorded and does not fail the gate
(the DMA-heap round is the pass criterion), but a real run must satisfy the
same raw-counter gate as the main round, and an asymmetric SKIP (one side
ran, the other skipped) is a failure.

### GPU kind selects the GEM domain

The amdgpu exporter pins the BO into GTT when a foreign importer (the NHI)
maps it — so the transport always targets system memory — but the *starting*
domain must be chosen per GPU kind (`--gpu-kind auto|igpu|dgpu`, default
auto):

- **iGPU (APU)** — its VRAM heap is a BIOS carve-out (0.5 GiB on the
  Strix-Halo rigs). Carve-out BOs cannot be pinned to GTT for the NHI; the
  first map fails `-EINVAL`. The bench allocates **GTT-domain** BOs directly
  — on an APU the same physical memory as the carve-out, and what ROCm AI
  frameworks prefer anyway.
- **dGPU** — dedicated GDDR/HBM. The bench allocates **VRAM-domain** BOs
  (the real GPU memory an RCCL tensor lives in); amdgpu migrates the BO to
  GTT on the NHI map. If the exporter still refuses VRAM on the first
  transfer, the bench retries the run once with GTT placement before
  declaring SKIP.

The VRAM heap size is the detection signal (`< 4 GiB` → iGPU; the IGP is not
always at bus 00:00.0 — Strix-Halo hangs it at `0000:c5:00.0`). The
first-transfer-only `-EINVAL` from a **GTT**-placed BO (a stack limitation,
not an OdinLink bug) is still a clean SKIP with an explicit reason; anything
after the first transfer, or any other errno, remains a hard failure.
(Observed on a Strix-Halo APU: VRAM BOs refuse to map regardless of IOMMU
mode, while GTT placement maps and transports fine, but only under a
**translated IOMMU** (`iommu=on`): under `iommu=pt`/`off` the NHI never
completes RX DMA to GTT buffers — `completed=0` ring timeouts. So the rigs
boot `iommu=on`.) With ROCm absent, `--allocator hip` always SKIPs on such
rigs.

The raw-geometry reasoning applies unchanged: amdgpu-exported BOs (GTT or
VRAM) carry page-granular SG tables like the DMA heap, so `ODL_TB5_RAW_CELL_MAX`
(2048) keeps them raw-eligible by construction.

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
