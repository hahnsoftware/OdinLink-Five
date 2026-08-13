# Native RCCL over OdinLink InfiniBand

This is the validated native RCCL path for the two Strix Halo nodes. It uses RCCL's built-in `NET/IB` transport through the OdinLink verbs provider. It does not load the legacy OdinLink RCCL plugin.

## Validated result

On 2026-07-13, a two-rank `all_reduce_perf` run across `ai1` and `ai2` completed every size from 8 bytes through 1 MiB with zero wrong values.

RCCL reported:

```text
RCCL version: 2.28.3-develop_deprecated:57e5868
NET/IB: odl_tb5_0:1/IB speed=20000
Initialized NET plugin IB
Using network IB
Connected all rings
Connected all trees
Out of bounds values: 0 OK
```

The short correctness run reached about 1.15 GB/s at 1 MiB. This is not yet a tuned maximum-bandwidth measurement.

## Why a source build is required

The packaged RCCL 2.27.7 build `96a25b5` initialized both ranks but crashed in HIP on the first collective. The same crash happened with Ethernet sockets, proving it was below OdinLink.

The working library uses:

- ROCm runtime 7.2.4
- RCCL source commit `57e58688f44c77076ad536ef1f6b68741fc6e694`
- ROCm 7.2.0 compiler build `26014`
- collective tracing disabled because gfx1151 does not expose the hardware register used by that optional tracer
- output SHA-256 `bb4d2c0ee6a07f3b2dc70535ab342a10011cab06137e879822f9ba62a699c9d5`

This follows the compiler workaround confirmed in ROCm issue #6074, with the additional gfx1151 tracing switch.

## Obtain the compiler without changing ROCm

Download and extract the older compiler beside the installed runtime:

```bash
wget https://repo.radeon.com/rocm/apt/7.2/pool/main/r/rocm-llvm/rocm-llvm_22.0.0.26014.70200-43~22.04_amd64.deb \
  -O /tmp/rocm-llvm-7.2.0.deb

echo '88d604fe0ab4a2502f0c7e0f8d3b7b49e829ae3057971de681a68b2fdb2505d2  /tmp/rocm-llvm-7.2.0.deb' \
  | sha256sum -c -

mkdir -p /root/llvm-7.2.0
dpkg-deb -x /tmp/rocm-llvm-7.2.0.deb /root/llvm-7.2.0
```

## Build RCCL for gfx1151

Check out the recorded RCCL source revision, then run the repository helper:

```bash
git clone https://github.com/ROCm/rccl.git /root/rccl-src
git -C /root/rccl-src checkout 57e58688f44c77076ad536ef1f6b68741fc6e694

RCCL_SRC=/root/rccl-src \
LLVM_ROOT=/root/llvm-7.2.0/opt/rocm-7.2.0/llvm \
JOBS=12 \
  ./scripts/build_rccl_gfx1151.sh
```

The final device link is quiet and took about 35 minutes on `ai1`; its linker used roughly 9.6 GB RAM. Do not assume it is hung while CPU time is still increasing.

Copy the resulting `build/release/librccl.so.1.0` to the same path on both nodes and create the normal symlinks. Verify that both copies have the same SHA-256.

## Build and install the OdinLink native verbs bridge

On both nodes, from this repository:

```bash
cmake -S . -B build -DBUILD_VERBS=ON
cmake --build build --target \
  odl_tb5_verbs \
  odl_tb5_verbs_dlopen_bridge \
  test_verbs_rccl_probe \
  test_verbs_dlopen_probe \
  -j"$(nproc)"
```

Run both probes before RCCL:

```bash
LD_LIBRARY_PATH=build/verbs:build/lib \
LD_PRELOAD=build/verbs/libodl_tb5_verbs.so \
  build/verbs/tests/test_verbs_rccl_probe

LD_LIBRARY_PATH=build/verbs:build/lib \
LD_PRELOAD=build/verbs/libodl_tb5_verbs_dlopen_bridge.so \
ODL_TB5_VERBS_DLOPEN_BRIDGE_ALL=1 \
ODL_TB5_VERBS_LIBRARY=build/verbs/libodl_tb5_verbs.so.0 \
  build/verbs/tests/test_verbs_dlopen_probe
```

Both nodes must report an active 20 Gb/s IB port and a complete private verbs handle.

## Run the two-node collective

The example below assumes passwordless SSH between the containers and the same paths on both. Ethernet is used only for MPI and RCCL setup; bulk collective traffic uses `odl_tb5_0`.

```bash
export LD_LIBRARY_PATH=/root/rccl-fixed:/root/OdinLink-Five/build/verbs:/root/OdinLink-Five/build/lib:/opt/rocm/lib
export LD_PRELOAD=/root/OdinLink-Five/build/verbs/libodl_tb5_verbs_dlopen_bridge.so
export ODL_TB5_VERBS_LIBRARY=/root/OdinLink-Five/build/verbs/libodl_tb5_verbs.so.0
export NCCL_NET=IB
export NCCL_IB_HCA=odl_tb5_0
export NCCL_SOCKET_IFNAME=eth0
export NCCL_DEBUG=INFO

mpirun --allow-run-as-root \
  -np 2 -H ai1:1,ai2:1 \
  -x LD_LIBRARY_PATH \
  -x LD_PRELOAD \
  -x ODL_TB5_VERBS_LIBRARY \
  -x NCCL_NET \
  -x NCCL_IB_HCA \
  -x NCCL_SOCKET_IFNAME \
  -x NCCL_DEBUG \
  /root/rccl-tests/build/all_reduce_perf \
    -b 8 -e 1M -f 2 -g 1 -n 5 -w 1
```

Do not accept communicator initialization alone as success. The result must say `Using network IB`, complete the data table, report zero wrong values, and destroy both communicators cleanly.

## Current performance boundary

RCCL currently logs `GPU Direct RDMA Disabled for HCA odl_tb5_0`. The successful native path therefore stages data through host memory. The OdinLink provider already supports DMA-buf memory, but RCCL cannot associate the synthetic verbs device with the GPU because `/sys/class/infiniband/odl_tb5_0/device` does not exist. Enabling that zero-copy path is the next throughput step.

Stock llama.cpp also does not create a multi-process RCCL communicator: its HIP/RCCL option initializes GPUs within one process. Two-node llama.cpp inference therefore remains on the proven RPC-RDMA path until llama.cpp gains a distributed RCCL execution layer. The native collective validated here is the required transport foundation for that work.