# OdinLink Benchmarks

All numbers are measured point-to-point between two real machines over a
Thunderbolt / USB4 cable. Each section lists the exact commands so you can
reproduce the run on your own hardware and compare.

## Test environment

| | |
|---|---|
| Machines | 2 × AMD Ryzen AI MAX+ 395 ("Strix Halo") |
| Link | USB4 v1, negotiated **20 Gb/s × 2 lanes** per direction |
| Kernel | 7.0.14-3-pve |
| DMA paths | 2 (NHI ring budget caps usable paths at 2 on Strix Halo; a 3rd ring pair fails to allocate) |

Throughout, **S1 = server**, **S2 = client**. Replace `<server-ip>` with the
server's IP on the management network (the data itself always travels over the
Thunderbolt link, never the IP network). "Goodput" is application-level bytes
received, not the TX submission rate.

> **Reproducibility notes**
> - The verbs dmabuf ping-pong server is *one-shot with a fixed sweep* — the
>   server and client must be started with **identical** `--sizes/--iters/--warmup`,
>   or the ping-pong desyncs.
> - `ib_send_bw`/`ib_send_lat` (host path) arm the stream RX pool on all paths.
>   Run the **dmabuf benchmark before perftest**, or reload the module between
>   them, otherwise the pooled RX frames collide with dmabuf receives.
> - Numbers vary a few percent run-to-run; take the median of several runs.

## Building & loading

```bash
sudo apt install build-essential cmake linux-headers-$(uname -r) \
     libibverbs-dev rdma-core pkg-config perftest ibverbs-utils
git clone https://github.com/Geramy/OdinLink-Five.git
cd OdinLink-Five && mkdir -p build && cd build
cmake .. -DBUILD_VERBS=ON && make -j$(nproc)
cd ..

# Load the module on BOTH machines (never rmmod on an active link — reboot instead).
# On a Secure Boot machine, sign odl_tb5.ko first with your enrolled MOK.
sudo insmod driver/odl_tb5.ko
```

The character device `/dev/odl_tb5_0` and `2 DMA paths enabled` / `entering
READY state` appear in `dmesg` once both ends complete the XDomain handshake.

---

## 1. Stream throughput — CLI (single path vs. MIMO)

The `odl_tb5_cli` bandwidth/MIMO tests drive the host-staged **stream** path.
A single stream stays on one DMA path; MIMO opens several streams that stripe
across both paths (the intended shape for NCCL/RCCL channel parallelism).

**Server (S1):**
```bash
build/cli/odl_tb5_cli server -d 0
```

**Client (S2):**
```bash
# Single-stream bandwidth (one DMA path)
build/cli/odl_tb5_cli client -d 0 -t bandwidth

# MIMO — 4 streams across 2 DMA paths
build/cli/odl_tb5_cli client -d 0 -t mimo --streams 4

# Idle round-trip latency
build/cli/odl_tb5_cli client -d 0 -t latency

# Latency under a concurrent 1 MB bulk load
build/cli/odl_tb5_cli client -d 0 -t latency-load
```

| Configuration | Throughput | Goodput | Notes |
|---|---|---|---|
| Single stream, 1 DMA path | **9.3 Gb/s** (1.16 GB/s) | 100% | Per-path ceiling — router flow-control, not CPU or window |
| 4 streams, **2 DMA paths** (MIMO) | **17.9 Gb/s** (2.24 GB/s) | 100% | Even ~50/50 stripe, 1.93× single path, 0 drops |
| Idle latency (64 B round-trip) | **21.9 µs** median | — | Unchanged by multi-path |
| Latency under 1 MB bulk load | 958 µs median | — | ⚠️ Head-of-line blocking (small messages queue behind bulk) — being worked on |

The ~9.3 Gb/s cap is *per DMA path* (router credits), so aggregate throughput
scales with parallel ring/HopID pairs. Stripe width is the module param
`odl_num_paths` (default 2).

---

## 2. Zero-copy DMA-buf over verbs — the GPU / RCCL transport

Symmetric zero-copy `ibv_reg_dmabuf_mr` **send and receive** — the path an
RCCL/GPU workload actually takes — measured with a two-box verbs ping-pong over
real `/dev/dma_heap/system` buffers. The DMA path through the NHI is identical
for a GPU (amdgpu/CUDA) dmabuf; the CPU-backed heap just makes it runnable
without a GPU. Every run verifies the bytes across the link against a
position-dependent pattern (integrity OK), so a no-op transport shows up as a
failure rather than a bogus throughput number.

A single transfer is **block-striped** across the negotiated DMA paths (each
path owns one contiguous half of the buffer), which reaches the same aggregate
as the MIMO stream path while staying zero-copy.

**Build the benchmark** (not part of CMake — links the provider directly):
```bash
gcc -O2 -o bench_verbs_dmabuf verbs/tests/bench_verbs_dmabuf.c \
    -Iverbs/src -Lbuild/verbs -Lbuild/lib -Wl,-rpath-link,build/lib \
    -lodl_tb5_verbs -libverbs -lpthread
```

**Server (S1):**
```bash
LD_LIBRARY_PATH=build/verbs:build/lib ./bench_verbs_dmabuf server
```

**Client (S2)** — must use the same sweep as the server (defaults shown):
```bash
LD_LIBRARY_PATH=build/verbs:build/lib ./bench_verbs_dmabuf client \
    --sizes 65536,262144,1048576,4194304,8388608 --iters 200 --warmup 20
```

### Results — 2 DMA paths (block-striped, default)

| Transfer size | One-way throughput | Median RTT | Integrity |
|---|---|---|---|
| 64 KB  | 9.29 Gb/s (1.16 GB/s)  | 97 µs   | OK |
| 256 KB | 15.12 Gb/s (1.89 GB/s) | 275 µs  | OK |
| 1 MB   | 17.18 Gb/s (2.15 GB/s) | 975 µs  | OK |
| 4 MB   | 17.45 Gb/s (2.18 GB/s) | 3.84 ms | OK |
| 8 MB   | **17.56 Gb/s** (2.20 GB/s) | 7.64 ms | OK |

Large transfers reach ~17.5 Gb/s — the same aggregate as the 2-path MIMO
stream path (17.9 Gb/s), now over the true zero-copy GPU path.

### Single-path comparison

The stripe width can be capped for A/B comparison with the writable module
param `odl_dmabuf_paths` (`0` = use all negotiated paths). **Set it identically
on both machines**, then rerun the client above:

```bash
# on BOTH machines
echo 1 | sudo tee /sys/module/odl_tb5/parameters/dmabuf_paths   # force single path
# ...run the benchmark...
echo 0 | sudo tee /sys/module/odl_tb5/parameters/dmabuf_paths   # restore multipath
```

| Transfer size | 1 path (`dmabuf_paths=1`) | 2 paths (default) |
|---|---|---|
| 64 KB  | 5.91 Gb/s  | 9.29 Gb/s  |
| 256 KB | 8.23 Gb/s  | 15.12 Gb/s |
| 1 MB   | 9.57 Gb/s  | 17.18 Gb/s |
| 4 MB   | 9.74 Gb/s  | 17.45 Gb/s |
| 8 MB   | 9.81 Gb/s  | **17.56 Gb/s** |

Both configurations pass integrity at every size; 2 paths is ~1.8× single path
at 8 MB. (Striping is done as contiguous per-path blocks, not frame-interleaved
round-robin — interleaving has two DMA engines writing cache-line-adjacent
memory at full rate, which corrupts frame-boundary bytes.)

---

## 3. Stock RDMA perftest over the provider (host-memory path)

The unmodified `perftest` tools run end-to-end through the OdinLink verbs
provider, proving standard-tooling compatibility. This exercises the
**host-memory** path (`ibv_reg_mr` → kernel copy), not the zero-copy dmabuf
path — it is single-path (`paths[0]`).

The provider is not installed system-wide; it is interposed via `LD_PRELOAD`:

```bash
export LD_LIBRARY_PATH=build/lib:build/verbs
export LD_PRELOAD=build/verbs/libodl_tb5_verbs.so
ibv_devinfo            # should now list odl_tb5_0 (finds nothing without LD_PRELOAD)
```

### Bandwidth — `ib_send_bw`

`ib_send_bw` needs `-t <= 64` (SQ depth) and `-n >= 5`.

**Server (S1):**
```bash
LD_LIBRARY_PATH=build/lib:build/verbs \
LD_PRELOAD=build/verbs/libodl_tb5_verbs.so \
  ib_send_bw -d odl_tb5_0 -s 65536 -n 2000 -t 32
```

**Client (S2):**
```bash
LD_LIBRARY_PATH=build/lib:build/verbs \
LD_PRELOAD=build/verbs/libodl_tb5_verbs.so \
  ib_send_bw -d odl_tb5_0 -s 65536 -n 2000 -t 32 <server-ip>
```

| bytes | iterations | BW peak | BW average | MsgRate |
|---|---|---|---|---|
| 65536 | 2000 | 6466.94 MiB/s | **1024.05 MiB/s** (~8.4 Gb/s) | 0.0164 Mpps |

### Latency — `ib_send_lat`

**Server (S1):**
```bash
LD_LIBRARY_PATH=build/lib:build/verbs \
LD_PRELOAD=build/verbs/libodl_tb5_verbs.so \
  ib_send_lat -d odl_tb5_0 -s 64 -n 1000
```

**Client (S2):**
```bash
LD_LIBRARY_PATH=build/lib:build/verbs \
LD_PRELOAD=build/verbs/libodl_tb5_verbs.so \
  ib_send_lat -d odl_tb5_0 -s 64 -n 1000 <server-ip>
```

| bytes | t_min | t_typical | t_avg | 99% | 99.9% |
|---|---|---|---|---|---|
| 64 | **4.85 µs** | 10.69 µs | 11.22 µs | 16.91 µs | 25.95 µs |

### One-sided RDMA — `ib_write_bw` / `ib_read_bw` / `ib_write_lat` / `ib_read_lat`

The provider emulates **RDMA WRITE, WRITE_WITH_IMM and READ** on top of the
two-sided stream transport by prefixing each message with a small operation
header (see `verbs/VERBS_PROVIDER.md`). The unmodified perftest one-sided tools
run end-to-end — same host-memory, single-path (`paths[0]`) transport as
`ib_send_bw`. Same `LD_PRELOAD` setup; swap the tool name:

```bash
# Server (S1) / Client (S2) — identical flags, client adds <server-ip>
ib_write_bw -d odl_tb5_0 -s 65536 -n 2000 -t 32 [<server-ip>]
ib_read_bw  -d odl_tb5_0 -s 65536 -n 2000 -t 32 [<server-ip>]
```

RDMA WRITE bandwidth (`ib_write_bw -a -t 32`, BW average, MiB/s):

| bytes | 1 KiB | 4 KiB | 16 KiB | 64 KiB | 256 KiB | 1 MiB | 8 MiB |
|---|---|---|---|---|---|---|---|
| WRITE | 463 | 732 | 842 | **948** | 1014 | 1026 | 1054 |
| READ  | 286 | 561 | 688 | 789 | 904 | 963 | **1046** |

WRITE peaks at ~1460 MiB/s (peak) / ~948 MiB/s (avg) at 64 KiB and settles at
~1054 MiB/s (~8.6 Gb/s) for large transfers — the same single-path host-memory
ceiling as `ib_send_bw`. READ trails WRITE (it is a request→response round-trip
over the same stream) but reaches ~1046 MiB/s at 8 MiB.

One-sided latency (4 KiB, `-n 1000`) — both tools poll the destination buffer
for the transferred bytes, so a passing run also confirms **data lands at the
correct remote address**:

| tool | t_min | t_typical | t_avg |
|---|---|---|---|
| `ib_write_lat` | **12.6 µs** | 20.97 µs | 19.30 µs |
| `ib_read_lat`  | **18.3 µs** | 28.91 µs | 27.29 µs |

> These are single-path host-memory numbers. The one-sided ops also work over
> zero-copy `ibv_reg_dmabuf_mr` regions (the RCCL path), which the responder
> places directly into the target dmabuf at the rkey-relative offset.
> **RDMA CM (`rping`, `-R` connection setup) is not implemented yet** — RCCL's
> IB transport and default perftest use their own TCP bootstrap, so CM is not on
> the RCCL critical path (see `verbs/VERBS_PROVIDER.md`).

---

*Last updated 2026-07-12. Hardware, kernel, cable quality and background load
all affect these numbers — treat them as a reference point, not a spec.*
