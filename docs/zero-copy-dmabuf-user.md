# Zero-Copy DMA-BUF Transfers — User Guide

OdinLink moves data between two machines over Thunderbolt 5. Most transfers
copy data through a bounce buffer in host RAM. **Zero-copy DMA-BUF transfers**
skip that copy: the data is read straight out of (or written straight into) a
DMA-BUF — typically GPU memory exported by ROCm (AMD) or CUDA (NVIDIA). This is
what lets the RCCL and NCCL plugins feed GPU tensors to the card without a
host-side staging copy.

Think of a DMA-BUF as a "window" the GPU has already handed to the OS. Instead
of copying your tensor into a separate send buffer, OdinLink points the
Thunderbolt DMA engine at that window directly.

## Prerequisites

- Kernel module `odl_tb5.ko` loaded (`sudo insmod driver/odl_tb5.ko`).
- A device node appears when a peer connects: `/dev/odl_tb5_0`, `/dev/odl_tb5_1`, …
  (Without a cable/peer, no node — unless loaded with `loopback=1`.)
- A DMA-BUF source. On bare metal this is usually a **DMA heap**
  (`/dev/dma_heap/*`); on a GPU box it is an AMDGPU- or CUDA-exported buffer.
  The test helpers fall back `dma_heap → amdgpu → memfd` automatically.
- For a real transfer you need **two** boxes cabled together (see Limitations).

## Using the C API

The library is `libodl_tb5`. The DMA-BUF calls are:

```c
/* One-shot (legacy double-buffer path) */
int odl_tb5_send_dmabuf(odl_tb5_t h, int dmabuf_fd, off_t offset, size_t len);
int odl_tb5_recv_dmabuf(odl_tb5_t h, int dmabuf_fd, off_t offset, size_t len);

/* Stream (multiplexed, kernel-managed) path */
int odl_tb5_stream_open(odl_tb5_t h, uint8_t filter_id, uint8_t *stream_id_out);
int odl_tb5_stream_send_dmabuf(odl_tb5_t h, uint8_t stream_id,
                               uint8_t dst_id, int dmabuf_fd,
                               uint64_t offset, uint64_t len);
int odl_tb5_stream_recv_dmabuf(odl_tb5_t h, uint8_t stream_id,
                               int dmabuf_fd, uint64_t offset, uint64_t len);
int odl_tb5_stream_close(odl_tb5_t h, uint8_t stream_id);
```

`dmabuf_fd` is a file descriptor for the exported buffer (obtained from the
GPU stack or a DMA heap). `offset`/`len` select the region to move. The
stream calls return `0` on success and a negative `errno` on failure
(`-110` = `ETIMEDOUT`, usually "peer did not post receive staging in time").

## Using with RCCL / NCCL

The plugins call the DMA-BUF API for you. Point the framework at the plugin:

```bash
# RCCL (AMD)
export RCCL_NET_PLUGIN=ODL_TB5
export RCCL_PLUGIN_DIR=/path/to/build/rccl

# NCCL (NVIDIA)
export NCCL_NET_PLUGIN=ODL_TB5
export NCCL_PLUGIN_DIR=/path/to/build/nccl
```

Requirements: CUDA 11.7+ (NCCL) or ROCm with GPU export (RCCL), `nvidia-drm`
modeset enabled for NCCL, and NCCL 2.12+ / RCCL build with the plugin.

## Validating your setup

These live in `build/tests/`:

| Test | What it proves | Topology |
|------|----------------|----------|
| `odl_tb5_test` | Device open, buffer info, poll, peer, send, and the RCCL plugin interface (3 suites) | single box |
| `odl_tb5_pair_dmabuf` | End-to-end legacy **and** stream DMA-BUF transfer with pattern verification | **two boxes** |
| `odl_stream_verify` | Multi-stream integrity (N streams × M rounds, content-checked) | **two boxes** |
| `odl_tb5_test_rccl_dmabuf` | DMA-BUF fd-plumbing / registration path | single box |

Run the single-box suite:

```bash
sudo chmod 666 /dev/odl_tb5_0
./build/tests/odl_tb5_test          # expect "ALL TESTS PASSED"
```

Two-box end-to-end (receiver arms RX **first**, then sender fires):

```bash
# box A (receiver)
./build/tests/odl_tb5_pair_dmabuf recv -d 1
# box B (sender), started after the receiver has posted staging
./build/tests/odl_tb5_pair_dmabuf send -d 1
```

Stream integrity:

```bash
# box A                          # box B
./odl_stream_verify server --dev 1   ./odl_stream_verify client --dev 1
```

## Known limitations

- **Single-box DMA-BUF starves by design.** Synchronous DMA-BUF traffic lives
  on reserved data rings; a TX frame only completes once the **peer** has posted
  RX staging. One box alone therefore times out (`-110`) on `send/recv_dmabuf`.
  This is expected — use the two-box `odl_tb5_pair_dmabuf` or
  `odl_stream_verify` to validate real transfers.
- The receiver must post its RX staging **before** the sender transmits. The
  paired test does this rendezvous automatically (it arms staging, then retries
  while the sender is started).
- `odl_tb5_test_rccl_dmabuf` exercises the fd-plumbing/registration path on one
  box; it intentionally does not complete a transfer without a peer.

See `docs/zero-copy-dmabuf-dev.md` for internals.
