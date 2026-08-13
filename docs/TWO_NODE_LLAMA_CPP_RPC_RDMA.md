# Two-node llama.cpp RPC-RDMA handoff

This document records the working state validated on 2026-07-13. It is the reproducible inference path while native multi-node RCCL is still being investigated.

## What works

`llama-cli` runs in `ai1`, loads the model from its read-only Hugging Face mount, and gives work to `ggml-rpc-server` in `ai2`. The ordinary Ethernet addresses are used to find the RPC server. Tensor data is then moved through the OdinLink verbs provider over the direct USB4 cable.

The working run created the connection on both nodes, moved it through its three ready states, processed the prompt at about 358 tokens/second, generated output, and shut down cleanly. This proves two-node inference, but it is not yet the maximum-speed design: llama.cpp currently opens one verbs queue, so only one of OdinLink's two DMA paths carries most of the traffic.

## Recorded machines

| Role | Proxmox host | Host IP | LXC | Container | Container IP |
|---|---|---:|---:|---|---:|
| llama.cpp client and model | `host-a` | `<host-a-ip>` | `102` | `ai1` | `<ai1-ip>` |
| llama.cpp RPC worker | `host-b` | `<host-b-ip>` | `102` | `ai2` | `<ai2-ip>` |

Both hosts were running kernel `7.0.14-3-pve`. Both containers had 12 CPU cores, 120000 MiB RAM, ROCm 7.2.4, and llama.cpp commit `e3546c794`.

The direct link reported two 10 Gb/s lanes, or 20 Gb/s in each direction. Earlier two-path zero-copy tests reached about 17.6 Gb/s. Do not unload the OdinLink kernel module merely to restart inference; older driver versions could hang during teardown.

## Required LXC devices

Container `102` on each host must receive these devices:

```ini
dev0: /dev/dri/card0,gid=44,mode=0666
dev1: /dev/dri/renderD128,gid=44,mode=0666
dev2: /dev/kfd,gid=44,mode=0666,uid=0
dev3: /dev/odl_tb5_0,mode=0666,uid=0
features: nesting=1
```

`ai1` also has the model collection mounted read-only:

```ini
mp0: /rpool/data/subvol-100-disk-0/models/huggingface,mp=/models/huggingface,ro=1
```

The OdinLink device appears only when the cable peer is connected. Before starting either program, check inside both containers:

```bash
ls -l /dev/kfd /dev/dri/renderD128 /dev/odl_tb5_0
```

## OdinLink userspace libraries

The following files were installed in both containers from this repository:

```text
/opt/odinlink/lib/libodl_tb5.so
/opt/odinlink/lib/libodl_tb5_verbs.so
```

The provider must include commit `3eaabbd` or later. That fix keeps receive requests alive for as long as the hardware may use them; without it, the worker can crash because it is handed an address that no longer belongs to a valid request.

## Build llama.cpp on both containers

Use the same llama.cpp revision on both nodes. The recorded revision is `e3546c794`.

```bash
cd /root/llama.cpp
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DGGML_HIP=ON \
  -DGGML_RPC=ON \
  -DGGML_RPC_RDMA=ON \
  -DGGML_HIP_RCCL=OFF
cmake --build build -j"$(nproc)" --target llama-cli rpc-server
```

Confirm the effective options:

```bash
grep -E 'GGML_(HIP|RPC|RPC_RDMA|HIP_RCCL):' build/CMakeCache.txt
```

Expected values are `HIP=ON`, `RPC=ON`, `RPC_RDMA=ON`, and `HIP_RCCL=OFF`. The resulting programs in the recorded build are:

```text
/root/llama.cpp/build/bin/llama-cli
/root/llama.cpp/build/bin/ggml-rpc-server
```

## Start the worker on ai2

Run this inside `ai2` and leave it in the foreground for the first reproduction so its log remains visible:

```bash
export LD_LIBRARY_PATH=/opt/odinlink/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export LD_PRELOAD=/opt/odinlink/lib/libodl_tb5_verbs.so
export GGML_RDMA_DEV=odl_tb5_0
export GGML_RDMA_GID=0

/root/llama.cpp/build/bin/ggml-rpc-server -H 0.0.0.0 -p 50052
```

From the Proxmox host, verify that the worker is listening:

```bash
pct exec 102 -- ss -ltnp
```

There is no persistent `llama-rpc-ai2.service` in the recorded state. Start the foreground process above, or create your own persistent unit after the manual run succeeds.

## Run inference on ai1

The model path below is a symlink into the Hugging Face cache mounted on `ai1`; the linked target must also exist. `ai2` does not need the model mount because the client sends it work over RPC.

```bash
export LD_LIBRARY_PATH=/opt/odinlink/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export LD_PRELOAD=/opt/odinlink/lib/libodl_tb5_verbs.so
export GGML_RDMA_DEV=odl_tb5_0
export GGML_RDMA_GID=0

MODEL=/models/huggingface/models/prithivMLmods/VibeThinker-3B-GGUF/VibeThinker-3B.Q4_K_M.gguf

/root/llama.cpp/build/bin/llama-cli \
  -m "$MODEL" \
  --rpc <ai2-ip>:50052 \
  -ngl 99 \
  -p "Write one short sentence about RDMA." \
  -n 16
```

## How to tell that RDMA was used

A successful run must satisfy all of these checks:

1. The worker listens on TCP port 50052 and the client reaches `<ai2-ip>:50052`.
2. Both program environments preload `/opt/odinlink/lib/libodl_tb5_verbs.so` and select `odl_tb5_0`.
3. The provider log shows the queue pass through states 1, 2, and 3. In plain English, that means created, connected to its peer, then ready to move data.
4. `llama-cli` reports prompt processing and produces text.
5. The worker remains alive after the request and the queue is destroyed cleanly when the client exits.

If the client silently falls back to an ordinary socket, first check the preload path, `GGML_RDMA_DEV`, and whether `/dev/odl_tb5_0` exists in both containers.

## Stopping and recovery

Stop the foreground worker with `Ctrl-C`. If a previous test left a worker behind, use this from `host-b`:

```bash
pct exec 102 -- pkill -TERM -x ggml-rpc-server
```

Do not reboot for a normal application restart. Reboot a host only if the OdinLink device itself is missing despite a connected peer or the driver is genuinely stuck.

## Current boundary: RCCL is separate

This working recipe deliberately has `GGML_HIP_RCCL=OFF`. llama.cpp's current HIP/RCCL option creates all GPUs inside one process, so it is not the working cross-node path described here.

Native RCCL over the standard verbs path now works across both nodes. A rebuilt RCCL 2.28.3 completed all-reduce collectives from 8 bytes through 1 MiB over `NET/IB` with zero wrong values; see [Native RCCL over OdinLink InfiniBand](NATIVE_RCCL_IB.md). This fixes the earlier packaged-RCCL crash, but stock llama.cpp still has no cross-process RCCL execution layer. RPC-RDMA therefore remains the reproducible two-node llama.cpp inference path while native RCCL is the validated transport foundation for future distributed execution.

## Known performance next step

The transport and cable can use two independent paths, but llama.cpp RPC-RDMA currently creates one queue. The next performance change should either let llama.cpp open multiple queues or stripe one logical transfer across both paths. Re-run correctness tests before comparing bandwidth or tokens/second.
