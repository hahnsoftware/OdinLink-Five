# llama.cpp over OdinLink verbs

Validated path:

    ai1 (<ai1-ip>) -> host-a -> USB4 -> host-b -> ai2 (<ai2-ip>)

llama.cpp's GGML_HIP_RCCL option calls ncclCommInitAll. It only joins GPUs
visible to one process. Cross-node llama.cpp uses RPC-RDMA, which uses ordinary
InfiniBand verbs and the OdinLink verbs preload.

## Required build options

~~~text
GGML_HIP=ON
GGML_RPC=ON
GGML_RPC_RDMA=ON
~~~

GGML_HIP_RCCL is not required for this path.

## Container setup

Both containers need /dev/kfd, /dev/dri/renderD128, and /dev/odl_tb5_0. Install:

~~~text
/opt/odinlink/lib/libodl_tb5.so
/opt/odinlink/lib/libodl_tb5_verbs.so
~~~

OdinLink has no kernel uverbs device, so a normal rdma-core plugin cannot
discover it by itself. Both processes need:

~~~bash
export LD_LIBRARY_PATH=/opt/odinlink/lib
export LD_PRELOAD=/opt/odinlink/lib/libodl_tb5_verbs.so
export GGML_RDMA_DEV=odl_tb5_0
export GGML_RDMA_GID=0
~~~

The explicit GID is required because OdinLink exposes one synthetic, all-zero
GID while llama.cpp normally matches the GID to the TCP address.

## Run

On ai2:

~~~bash
ggml-rpc-server -H 0.0.0.0 -p 50052
~~~

On ai1:

~~~bash
llama-cli -m /models/huggingface/path/to/model.gguf   --rpc <ai2-ip>:50052 -ngl 99 -p "Hello"
~~~

ai2 does not need the model mount.

## Verified result

On 2026-07-13 this loaded VibeThinker-3B.Q4_K_M.gguf, activated a
reliable-connected verbs queue in both containers, processed a prompt, and
generated output. ai2 remained healthy after the client exited.

The USB4 link trained at 10 Gb/s per lane with two lanes. Existing zero-copy
tests reach about 17.5 Gb/s, close to that 20 Gb/s wire limit.

Current llama.cpp RPC traffic uses one queue at a time, which maps to one DMA
path. Startup also creates many short-lived RPC connections. The next
performance work is connection reuse and spreading concurrent queues across
both DMA paths; this result proves function, not final latency.
