# OdinLink-Five Security Audit Findings

**Date:** 2026-07-14  
**Scope:** Kernel driver (`driver/`), userspace verbs provider (`verbs/src/`), character device interface (`driver/odl_tb5_chardev.c`)  
**Focus:** Hot paths from kernel driver → userspace verbs transport exposing an IB verbs device

---

## Executive Summary

| Severity | Count |
|----------|-------|
| **Critical (P0)** | 4 |
| **High (P1)** | 3 |
| **Medium (P2)** | 3 |
| **Low (P3)** | 0 |

**Total Findings:** 10

---

## Critical Findings (P0)

### P0-1: Unbounded RX Assembly Buffer — Kernel OOM Risk

**Location:** `driver/odl_tb5_ring_dma.c:438-458`

**Code:**
```c
if (stream->rx_asm_len + payload_len > stream->rx_asm_cap) {
    size_t new_cap = max_t(size_t,
        8192,
        max(stream->rx_asm_cap * 2,
            stream->rx_asm_len + payload_len));
    void *nb = kmalloc(new_cap, GFP_ATOMIC);
    ...
}
```

**Issue:** A malicious peer can send a stream of `MSG_START` + many continuation frames without ever sending `MSG_END`. The assembly buffer grows exponentially (2× each frame) with **no upper bound**. Each frame allocates via `kmalloc(GFP_ATOMIC)` — uncontrollable kernel memory exhaustion.

**Impact:** Remote DoS → kernel OOM kill / system hang. No authentication on Thunderbolt XDomain connection.

**Fix:** Cap `rx_asm_cap` at `ODL_TB5_STREAM_PAYLOAD_MAX * MAX_FRAMES_PER_MSG` (e.g., 64 frames = 256 KB). Drop message and reset assembly on cap exceed.

---

### P0-2: Raw Pointer from `rkey` / `remote_addr` — RCE / Memory Corruption

**Location:** `verbs/src/odl_tb5_verbs_qp.c:301-307`

**Code:**
```c
if (mr && mr->mr_type == 1) {
    uint64_t off = (hdr->remote_addr >= mr->iova)
                 ? hdr->remote_addr - mr->iova : 0;
    ret = rx_dmabuf(qp, mr->dmabuf_fd, off, hdr->length);
} else if (mr) {
    ret = rx_msg(qp, (void *)(uintptr_t)hdr->remote_addr, hdr->length, NULL);
}
```

**Issue:** `hdr->remote_addr` comes from the **remote peer** (untrusted). For host MRs (`mr_type == 0`), it is cast directly to a userspace pointer and passed to `rx_msg` → `copy_to_user`. An attacker controlling the peer can write to **any address in the local process's address space**.

**Impact:** Arbitrary memory write in the verbs consumer process (e.g., RCCL, PyTorch, vLLM). Combined with ASLR bypass → RCE.

**Fix:** Validate `remote_addr` falls within the registered MR's `[addr, addr+length)` range. Reject if outside.

---

### P0-3: Batch Buffer Double-Free on Partial Submit Failure

**Location:** `driver/odl_tb5_ring_dma.c:2078-2093`

**Code:**
```c
for (i = 0; i < nframes; i++) {
    if (tb_ring_tx(...) < 0) {
        int unsub = nframes - i;
        atomic_sub(unsub, &batch->frames_pending);
        atomic_sub(unsub, &msg->frames_pending);
        msg->sent -= (size_t)unsub * ODL_TB5_STREAM_PAYLOAD_MAX;
        batch->total_frames = i;
        if (i == 0)
            odl_tb5_batch_pool_put(bpool, batch);  // <-- returns to pool
        ret = -EIO;
        goto wait_pending;
    }
}
```

**Issue:** If frame `i > 0` fails, frames `0..i-1` were already submitted to hardware. The batch is **not** returned to the pool (only `i==0` path does). But `wait_pending` → `tx_execute` error path → `kfree(msg)` → `odl_tb5_batch_pool_put(batch)` in `tx_batch_callback` when `frames_pending` hits zero. **Result:** batch returned to pool twice → double-free → kernel heap corruption.

**Fix:** On partial failure, mark batch as "errored" and let the completion callback return it once. Or: stop submitting on first error, wait for posted frames, then return batch.

---

### P0-4: Stream Use-After-Free on RCU Lookup / Destroy Race

**Location:** `driver/odl_tb5_ring_dma.c:1769-1787` (lookup) vs `1721-1740` (destroy)

**Code (lookup):**
```c
rcu_read_lock();
hash_for_each_possible_rcu(dev->streams, stream, node, stream_id) {
    if (stream->id == stream_id) {
        if (!kref_get_unless_zero(&stream->refcount))
            break;
        rcu_read_unlock();
        return stream;
    }
}
rcu_read_unlock();
```

**Code (destroy):**
```c
mutex_lock(&dev->stream_lock);
hash_del_rcu(&stream->node);
mutex_unlock(&dev->stream_lock);
...
ida_free(&dev->stream_ida, stream->id);
kref_put(&stream->refcount, odl_tb5_stream_free);
```

**Issue:** `hash_del_rcu` starts RCU grace period. `kref_put` may free immediately if refcount=1. Concurrent `stream_lookup` in RX callback (running in workqueue context) can:
1. Find stream in hash (not yet removed)
2. `kref_get_unless_zero` succeeds (refcount still 1)
3. `hash_del_rcu` completes, grace period ends
4. `kref_put` frees stream
5. Lookup returns freed pointer → use-after-free in RX callback

**Fix:** Use `kref_get_unless_zero` **before** `hash_del_rcu`, or use `rcu_head` callback for deferred free (call `kref_put` in RCU callback).

---

## High Findings (P1)

### P1-1: Busy-Poll `sched_yield()` Burns CPU Cores

**Location:** `verbs/src/odl_tb5_verbs_qp.c:47-58, 86-96, 144-150`

**Code:**
```c
static int tx_msg(...) {
    for (;;) {
        int ret = odl_tb5_stream_send(...);
        if (ret != -EAGAIN) return ret;
        if (!qp->worker_running) return -EINTR;
        sched_yield();  // <-- busy spin
    }
}
```

**Issue:** Send/recv workers spin on `sched_yield()` when device fd returns `EAGAIN`. Under load, this consumes **100% of a CPU core per QP**. Typical RCCL workloads create 1 QP per GPU → 8 cores burned per process.

**Fix:** Use `poll()`/`epoll()` on the device fd. The chardev `poll` op (`odl_tb5_poll`) already supports `EPOLLIN`/`EPOLLOUT` for stream completions. Block in `poll()` with timeout instead of yield.

---

### P1-2: Linear MR Lookup on Every RDMA Operation

**Location:** `verbs/src/odl_tb5_verbs_mr.c:119-133`

**Code:**
```c
pthread_mutex_lock(&ctx->mr_lock);
for (int i = 0; i < ctx->nmrs; i++) {
    struct odl_verbs_mr *mr = ctx->mrs[i];
    if (mr && (mr->base.rkey == rkey || mr->base.lkey == rkey)) {
        pthread_mutex_unlock(&ctx->mr_lock);
        return mr;
    }
}
pthread_mutex_unlock(&ctx->mr_lock);
```

**Issue:** Called from `handle_write` and `handle_read_req` in the **recv worker hot path** — once per RDMA WRITE/READ. With 100s of MRs (common in GPU collectives), this is a mutex contention + O(N) scan on every operation.

**Fix:** Hash table keyed by `rkey` (which equals `lkey` equals pointer address). Or `khash`/`uthash` in userspace.

---

### P1-3: RX Repost Spinlock Contention in Callback Context

**Location:** `driver/odl_tb5_ring_dma.c:2375-2406`

**Code:**
```c
void odl_tb5_rx_repost(struct odl_tb5_device *dev, int idx) {
    struct odl_tb5_path *path = &dev->paths[idx];
    while (atomic_read(&path->rx_posted) < target) {
        slot = odl_tb5_frame_pool_get(&dev->frame_pool);  // takes pool->lock
        if (!slot) { ODL_STAT_INC(...); break; }
        ...
        if (tb_ring_rx(path->rx.ring, &slot->frame) < 0) {
            ODL_STAT_INC(...);
            odl_tb5_frame_pool_put(&dev->frame_pool, slot);  // takes pool->lock
            break;
        }
        atomic_inc(&path->rx_posted);
    }
}
```

**Issue:** Called from **RX completion callback** (interrupt/softirq context) for every frame. Takes `frame_pool.lock` (spinlock) twice per frame. Under high RX throughput (multi-path, 4096 frames), this lock becomes a contention hotspot across CPUs.

**Fix:** Batch repost — collect N free slots under one lock acquisition, submit N frames, single atomic add. Or use per-CPU frame pools.

---

## Medium Findings (P2)

### P2-1: TX Adaptive Mode Switch Race

**Location:** `driver/odl_tb5_ring_dma.c:1800-1845`

**Code:**
```c
static enum odl_tb5_tx_mode odl_tb5_evaluate_tx_mode(...) {
    unsigned int pool_used = dev->frame_pool.size - dev->frame_pool.free_count;
    // No locking!
    if (dev->tx_adaptive.mode == ODL_TB5_TX_LATENCY) {
        if (pool_used + nframes > dev->tx_adaptive.high_watermark) {
            dev->tx_adaptive.mode = ODL_TB5_TX_THROUGHPUT;  // data race
        }
    } else {
        if (pool_used < dev->tx_adaptive.low_watermark) {
            if (++dev->tx_adaptive.consecutive_low >= ODL_TB5_MODE_HYSTERESIS) {
                dev->tx_adaptive.mode = ODL_TB5_TX_LATENCY;  // data race
            }
        }
    }
}
```

**Issue:** `pool_used`, `free_count`, `mode`, `consecutive_low`, `high_watermark`, `low_watermark` all accessed concurrently from multiple stream send paths **without locks or atomics**. Can cause:
- Mode thrashing (rapid flip-flop)
- Incorrect watermark evaluation
- Lost updates to `consecutive_low`

**Fix:** Use `atomic_t` for counters, `atomic_cmpxchg` for mode transition, or protect with a seqlock.

---

### P2-2: Ring-to-Context Linear Search on Every Callback

**Location:** `driver/odl_tb5_ring_dma.c:156-189`

**Code:**
```c
static struct odl_tb5_ring_ctx *odl_tb5_ring_to_ctx(struct tb_ring *ring) {
    struct odl_tb5_device *dev;
    int i;
    list_for_each_entry_rcu(dev, &odl_tb5_devices_list, list) {
        for (i = 0; i < dev->num_paths; i++) {
            if (dev->paths[i].tx.ring == ring) return &dev->paths[i].tx;
            if (dev->paths[i].rx.ring == ring) return &dev->paths[i].rx;
        }
    }
    return NULL;
}
```

**Issue:** Called from **every TX/RX completion callback** (thousands per second). Iterates global device list + paths. With multiple devices/paths, this is O(N) on hot path.

**Fix:** Store `struct odl_tb5_ring_ctx *` in `tb_ring->priv` (if available) or use a hash table keyed by ring pointer. The `ring_frame` already has `callback` — extend to carry context pointer.

---

### P2-3: Single SGE Limitation in Verbs Provider

**Location:** `verbs/src/odl_tb5_verbs_ops.c:558, 645, 756`

**Code:**
```c
struct ibv_sge *sge = (wr->num_sge > 0) ? &wr->sg_list[0] : NULL;
uint32_t len = sge ? sge->length : 0;
...
if (wr->num_sge < 0 || wr->num_sge > 1 || (wr->num_sge == 1 && !wr->sg_list)) {
    return EINVAL;
}
```

**Issue:** Only first SGE used; `num_sge > 1` rejected. Real IB verbs apps (MPI, RCCL) use scatter-gather for non-contiguous buffers. Forces extra memcpy or fails.

**Fix:** Implement SG list support in `odl_tx_desc` / `odl_recv_desc` — chain multiple frames or use DMA-buf scatter-gather.

---

## Recommended Fix Priority Order

| Order | Finding | Effort | Risk if Deferred |
|-------|---------|--------|------------------|
| 1 | P0-1: Unbounded RX assembly | Low (1 day) | Remote kernel DoS |
| 2 | P0-2: Raw pointer from peer | Medium (2 days) | RCE in user process |
| 3 | P0-4: Stream UAF on RCU race | Medium (2 days) | Kernel UAF / crash |
| 4 | P0-3: Batch buffer double-free | Low (1 day) | Kernel heap corruption |
| 5 | P1-1: Busy-poll CPU burn | Medium (3 days) | 100% CPU per QP |
| 6 | P1-2: Linear MR lookup | Low (1 day) | RDMA throughput collapse |
| 7 | P1-3: RX repost lock contention | Medium (2 days) | RX throughput ceiling |
| 8 | P2-1: TX mode race | Low (1 day) | Mode thrashing |
| 9 | P2-2: Ring-to-ctx linear search | Low (1 day) | Callback overhead |
| 10 | P2-3: Single SGE limit | Medium (3 days) | App compatibility |

---

## Testing Recommendations

1. **Fuzz RX assembly:** Send malformed stream frames (START without END, oversized payloads) via test harness
2. **MR bounds test:** Craft RDMA WRITE with `remote_addr` outside registered MR, verify rejection
3. **Stress stream create/destroy:** Concurrent open/close from multiple threads while RX traffic flows
4. **CPU profiling:** Run `ib_write_bw` with 8 QPs, measure CPU utilization (should be <10% per core, not 100%)
5. **Lock contention:** `perf record -g -e lock:lock_acquired` on RX heavy workload

---

## Appendix: Files Audited

| File | Lines | Focus |
|------|-------|-------|
| `driver/odl_tb5_ring_dma.c` | 2406 | DMA engine, frame pool, RX/TX callbacks, stream send/recv |
| `driver/odl_tb5_chardev.c` | 715 | ioctl handlers, mmap, poll, stream lifecycle |
| `driver/odl_tb5_core.h` | 600 | Core data structures, device/stream/ring definitions |
| `driver/uapi/odl_tb5_uapi.h` | 204 | ioctl ABI, stream headers, frame format |
| `verbs/src/odl_tb5_verbs_qp.c` | 829 | QP workers, RDMA opcode handling, send/recv |
| `verbs/src/odl_tb5_verbs_mr.c` | 159 | Memory registration, MR lookup |
| `verbs/src/odl_tb5_verbs_ops.c` | 517 | Symbol interposition, verbs dispatch table |
| `verbs/src/odl_tb5_verbs_main.c` | 236 | Device discovery, context creation |
| `verbs/src/odl_tb5_verbs.h` | ~500 | Internal types, rdma header, constants |

---

*Generated by automated code review. Manual verification recommended before applying fixes.*