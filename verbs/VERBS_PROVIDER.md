# OdinLink Verbs Provider — Technical Manual

## Overview

The OdinLink Verbs Provider (`libodl_tb5_verbs.so`) is a **libibverbs-compatible RDMA plugin** that exposes OdinLink-Five Thunderbolt 5 DMA transports through the standard Verbs API (`ibv_*`). It makes any verbs-aware application (NCCL, MPI, PyTorch DDP) use Thunderbolt 5 DMA without code changes.

```
┌─────────────────────────────────────────────────────┐
│                  Application                         │
│  (NCCL, MPI, PyTorch, ibv_* API)                    │
├─────────────────────────────────────────────────────┤
│              libibverbs (libibverbs.so.1)            │
├─────────────────────────────────────────────────────┤
│          libodl_tb5_verbs.so (symbol interposition) │
├─────────────────────────────────────────────────────┤
│       libodl_tb5.so (OdinLink-Five API)             │
├─────────────────────────────────────────────────────┤
│          Kernel: odl_tb5.ko (char device)            │
├─────────────────────────────────────────────────────┤
│          Thunderbolt 5 NHI DMA Engine                 │
└─────────────────────────────────────────────────────┘
```

## Operating Modes

### 1. Standalone Symbol Interposition (default)

The library provides its own `ibv_open_device()` that intercepts calls at link time. For OdinLink-Five devices, it creates a context backed by the real `odl_tb5` char device. For all other devices, it forwards to the real libibverbs.

**Usage:**
```bash
# Link directly
gcc -o my_app my_app.c -lodl_tb5_verbs -libverbs

# Or LD_PRELOAD for existing binaries
LD_PRELOAD=libodl_tb5_verbs.so mpirun --hostfile hosts ./my_mpi_app
```

### 2. rdma-core Provider Plugin

Built as `libodl_tb5-rdmav34.so` and installed into the libibverbs provider directory. Discovered automatically by `ibv_devinfo` and `ibv_open_device`.

**Install:**
```bash
sudo cp build/verbs/libodl_tb5-rdmav34.so /usr/lib/aarch64-linux-gnu/libibverbs/
ibv_devinfo  # Should show odl_tb5 devices
```

### 3. Hardware Simulation / Mock

A user-space mock library simulates two TB5 peers connected via shared memory. No kernel module or Thunderbolt cable required.

**Usage:**
```bash
# Create a fake /dev/odl_tb5_0 for the verbs provider to discover
mkfifo /dev/odl_tb5_0

# Run with mock intercepting hardware calls
LD_PRELOAD=libodl_tb5_mock.so \
LD_LIBRARY_PATH=build/verbs:build/lib \
./my_verbs_app
```

## API Mapping

| Verbs API | OdinLink-Five Mapping | Zero-Copy? |
|-----------|----------------------|------------|
| `ibv_open_device` | `odl_tb5_open` | N/A |
| `ibv_close_device` | `odl_tb5_close` | N/A |
| `ibv_alloc_pd` | lightweight struct alloc | N/A |
| `ibv_reg_mr` | host memory pinning | ❌ (memcpy) |
| `ibv_reg_dmabuf_mr` | DMA-buf fd passthrough | ✅ (GPU zero-copy) |
| `ibv_create_cq` | completion ring + eventfd | N/A |
| `ibv_create_qp` | `odl_tb5_stream_open` | N/A |
| `ibv_post_send` (SEND) | enqueue → worker → `stream_send` | ✅ (async) |
| `ibv_post_send` (RDMA WRITE / WRITE_WITH_IMM) | header + payload frame → peer places at `remote_addr` | ✅ (async) |
| `ibv_post_send` (RDMA READ) | READ_REQ header → peer replies READ_RESP + payload | ✅ (async) |
| `ibv_post_recv` | RQ ring, drained by recv worker | ❌ |
| `ibv_poll_cq` | dequeue from eventfd ring | N/A |
| `ibv_modify_qp` | stream state tracking + peer stream id | N/A |

Supported `ibv_post_send` opcodes: `IBV_WR_SEND`, `IBV_WR_RDMA_WRITE`,
`IBV_WR_RDMA_WRITE_WITH_IMM`, `IBV_WR_RDMA_READ`. `IBV_SEND_SIGNALED` is
honoured; `max_send_sge`/`max_recv_sge` are 1 and inline data is unsupported.

## Async Completion Model

```
ibv_post_send(qp, wr, NULL)
    │
    ▼
enqueue wr → per-QP SQ ring     ← returns immediately (non-blocking)
    │
    ▼
worker thread detects new wr
    │
    ├── dmabuf MR? → odl_tb5_stream_send_dmabuf()  ← zero-copy GPU
    └── host MR?   → odl_tb5_stream_send()         ← kernel copies data
    │
    ▼
post struct ibv_wc → CQ ring
    │
    ├── eventfd_write()     ← wakes ibv_get_cq_event()
    └── ibv_poll_cq()       ← drains from CQ ring
```

## One-Sided RDMA over a Two-Sided Transport

The NHI stream transport is **two-sided** (SEND/RECV) and has no concept of a
remote address. RDMA WRITE/READ — which `ib_write_bw`/`ib_read_bw` and RCCL's
IB net transport require — are emulated by prefixing every stream message with a
small operation header (`struct odl_rdma_hdr`). Because the transport preserves
message boundaries (1 `stream_send` == 1 `stream_recv`), the responder reads the
header first and dispatches on the opcode. Each verb is one header message,
optionally followed by one payload message:

```
initiator                         responder (recv worker = dispatcher)
─────────                         ────────────────────────────────────
RDMA WRITE   [hdr WRITE|rkey|va] → look up local MR by rkey, recv payload
             [payload]           →   straight into remote_addr  (no completion)

WRITE_IMM    [hdr WRITE_IMM|imm] → place payload, then consume an RQ WR and
             [payload]           →   post IBV_WC_RECV_RDMA_WITH_IMM(imm)

RDMA READ    [hdr READ_REQ|rkey] → read local MR, enqueue READ_RESP on the
                                 ←   send worker: [hdr READ_RESP][payload]
             place payload in local buffer, complete IBV_WC_RDMA_READ

SEND         [hdr SEND]          → consume an RQ WR, recv payload,
             [payload]           →   post IBV_WC_RECV
```

Key points:

- **rkey → MR reverse map.** Every MR (host and dmabuf) gets a unique non-zero
  `lkey`/`rkey`. `odl_find_mr_by_rkey()` maps the wire rkey back to the local MR
  so the responder knows where `remote_addr` lands. For host MRs `remote_addr`
  is directly a valid pointer into the responder's own registered buffer; for
  dmabuf MRs it is the rkey-relative offset into the target dmabuf (zero-copy).
- **All TX on one thread.** The send worker is the only thread that transmits,
  so the `[hdr][payload]` pair is never interleaved with another op. The recv
  worker answers a READ by *enqueuing* a READ_RESP descriptor onto the send
  worker rather than transmitting itself.
- **Passive target.** One-sided WRITE/READ need no posted receive on the target
  — the recv worker always reads incoming headers, so it services them whether
  or not the application posted anything.
- **Both ends run this provider**, so the framing is private and self-consistent
  (header fields are native little-endian; both test boxes are x86-64).

### Bootstrap ordering caveat

A *simultaneous* bidirectional first-contact SEND — both peers posting a send on
a fresh QP before either has received anything — can drop one message on this
transport. Real apps never do this: perftest and RCCL/NCCL exchange QP
parameters over their own TCP bootstrap, and ping-pong benchmarks alternate
directions. The `test_verbs_write_imm` rendezvous is therefore ordered
(client sends first, server replies). RDMA WRITE/READ are unaffected — only the
initial QP-level SEND handshake needs to avoid a dead heat.

### Not yet implemented: RDMA CM

`rdma_cm`/`librdmacm` connection management (`rping`, perftest `-R`) is **not**
implemented. It is deliberately lowest priority: RCCL/NCCL's IB transport and
default perftest exchange QP information (qpn, psn, rkey, VA) over their **own
TCP bootstrap** and connect with `ibv_modify_qp` — they never call into
`librdmacm`. A future CM shim would interpose the `rdma_*` entry points and run a
small TCP-based rendezvous that drives the same `ibv_modify_qp` RESET→INIT→RTR→
RTS sequence the ibverbs path already uses.

## Device Discovery

Devices are discovered by scanning `/dev/odl_tb5_N` entries. Each entry becomes an `ibv_device` that can be opened with `ibv_open_device`.

```c
#include <odl_tb5/odl_tb5_verbs_wrapper.h>

int ndev = odl_num_tb5_devices();
struct ibv_device *dev = odl_find_tb5_device(0);
struct ibv_context *ctx = ibv_open_device(dev);
```

## Debugging

Set `ODL_VERBS_DEBUG` environment variable:

| Level | Output |
|-------|--------|
| 0 | Off (default) |
| 1 | Errors only |
| 2 | + Warnings |
| 3 | + Info (device opens, etc.) |
| 4 | + Verbose (send/recv ops) |
| 5 | + Trace (all function entry/exit) |

```bash
ODL_VERBS_DEBUG=5 LD_PRELOAD=libodl_tb5_verbs.so ./my_app
```

## Zero-Copy GPU Memory

GPUDirect RDMA is achieved through `ibv_reg_dmabuf_mr()`:

```c
// 1. Export GPU memory as a dmabuf file descriptor
int dmabuf_fd = export_cuda_dmabuf(gpu_ptr, size);

// 2. Register with verbs — same API as Apple's ibv_reg_dmabuf_mr
struct ibv_mr *mr = ibv_reg_dmabuf_mr(pd, 0, size, 0, dmabuf_fd,
                                       IBV_ACCESS_LOCAL_WRITE);

// 3. Post send — dmabuf fd passed to kernel driver
struct ibv_send_wr wr = {
    .sg_list = &(struct ibv_sge){.lkey = mr->lkey, .length = size},
    .num_sge = 1,
    .opcode = IBV_WR_SEND,
};
ibv_post_send(qp, &wr, NULL);
```

## NCCL Integration

NCCL's built-in Verbs transport discovers the OdinLink-Five provider automatically when the provider plugin is installed:

```bash
# Install provider
sudo cp build/verbs/libodl_tb5-rdmav34.so /usr/lib/aarch64-linux-gnu/libibverbs/

# NCCL uses it automatically via verbs
NCCL_DEBUG=INFO torchrun --nproc_per_node=1 --nnodes=2 \
    --node_rank=0 --master_addr=192.168.1.1 --master_port=12345 \
    train.py
```

## Native RCCL integration

RCCL's built-in `NET/IB` transport opens `libibverbs` privately. A normal
`LD_PRELOAD=libodl_tb5_verbs.so` does not affect calls looked up through that
private handle. The small dlopen bridge redirects only RCCL/NCCL's private
`libibverbs` load to the standalone OdinLink provider. It does not select the
legacy OdinLink RCCL plugin.

Build and install both libraries on every rank:

```bash
cmake --build build --target odl_tb5_verbs odl_tb5_verbs_dlopen_bridge -j"$(nproc)"
sudo cmake --install build --component verbs
sudo ldconfig
```

Run RCCL with its standard IB transport:

```bash
export LD_PRELOAD=/usr/local/lib/libodl_tb5_verbs_dlopen_bridge.so
export ODL_TB5_VERBS_LIBRARY=/usr/local/lib/libodl_tb5_verbs.so.0
export NCCL_NET=IB
export NCCL_IB_HCA=odl_tb5_0
export NCCL_DEBUG=INFO
```

`ODL_TB5_VERBS_LIBRARY` is optional when the provider is in the dynamic
linker's normal search path. The bridge falls back to the real `libibverbs` if
the OdinLink provider cannot be opened. Set
`ODL_TB5_VERBS_DLOPEN_BRIDGE_ALL=1` only for the private-handle probe below;
normal applications should leave it unset so unrelated libraries keep using
the system provider.

Hardware readiness and RCCL's private lookup can be checked independently:

```bash
LD_PRELOAD=build/verbs/libodl_tb5_verbs.so \
  build/verbs/tests/test_verbs_rccl_probe

LD_LIBRARY_PATH=build/verbs:build/lib \
LD_PRELOAD=build/verbs/libodl_tb5_verbs_dlopen_bridge.so \
ODL_TB5_VERBS_DLOPEN_BRIDGE_ALL=1 \
ODL_TB5_VERBS_LIBRARY=build/verbs/libodl_tb5_verbs.so.0 \
  build/verbs/tests/test_verbs_dlopen_probe
```

The first test checks that the cable device looks like an active 20 Gb/s IB
port. The second checks every versioned verbs entry point that RCCL resolves
and confirms that its private handle can see `odl_tb5_0`.
## Linux ↔ macOS Compatibility

The verbs provider on Linux creates the same `ibv_*` API surface that Apple's `libthunderboltrdma.dylib` provides on macOS. If the NHI DMA ring protocol is wire-compatible, the same application code runs on both platforms without changes.

| Platform | Provider | Kernel Driver |
|----------|----------|---------------|
| Linux | `libodl_tb5-rdmav34.so` | `odl_tb5.ko` |
| macOS | `libthunderboltrdma.dylib` | `AppleThunderboltRDMA.kext` |

## Build

```bash
mkdir build && cd build
cmake .. -DBUILD_VERBS=ON
make -j$(nproc) odl_tb5_verbs
```

## Test

```bash
# With mock (no hardware required)
mkfifo /dev/odl_tb5_0
LD_PRELOAD=libodl_tb5_mock.so \
LD_LIBRARY_PATH=build/verbs:build/lib \
build/verbs/tests/test_verbs_mock_loopback

# With real hardware
sudo insmod driver/odl_tb5.ko
build/verbs/tests/test_verbs_basic
```

## File Layout

```
verbs/
├── CMakeLists.txt                    # Build configuration
├── VERBS_PROVIDER.md                 # This file
├── src/
│   ├── odl_tb5_verbs.h               # Internal header
│   ├── odl_tb5_verbs_debug.h         # Debug logging + assertions
│   ├── odl_tb5_verbs_main.c          # Device scan + ibv_open_device interposition
│   ├── odl_tb5_verbs_device.c        # Context lifecycle + queries
│   ├── odl_tb5_verbs_pd.c            # Protection domains
│   ├── odl_tb5_verbs_mr.c            # Memory regions (+ dmabuf)
│   ├── odl_tb5_verbs_cq.c            # Completion queues + eventfd
│   ├── odl_tb5_verbs_qp.c            # Queue pairs + async workqueue
│   ├── odl_tb5_verbs_ops.c           # ibv_context ops dispatch table
│   └── odl_tb5_verbs_wrapper.h       # Public wrapper API header
├── tests/
│   ├── test_verbs_basic.c            # Hardware smoke test
│   ├── test_verbs_mock_loopback.c    # Mock loopback test
│   └── odl_tb5_verbs_mock.c          # Mock library (LD_PRELOAD)
```
