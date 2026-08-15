// SPDX-License-Identifier: MIT
/*
 * OdinLink — The DMA Engine: Sending and Receiving Packets
 *
 * The Thunderbolt NHI (Native Host Interface) gives us a ring of fixed-size
 * DMA slots — think of it like a circular conveyor belt of 4KB bins. You
 * drop data into a bin on the TX belt, the hardware ships it across the
 * cable, and the other end picks it up from their RX belt.
 *
 * This file handles:
 *   - Allocating those DMA rings and the buffer memory behind them
 *   - A "frame pool" of reusable 4KB slots (no re-allocation between sends)
 *   - Two send modes: latency (one slot at a time, low delay) and throughput
 *     (big 256KB batches, high bandwidth)
 *   - RX assembly — the other side may split a message across multiple 4KB
 *     frames; this reconstructs them into a single buffer
 *   - Callbacks that fire when the hardware finishes a TX or completes an RX
 *
 * Also handles DMA-buf (GPU memory) transfers for zero-copy GPU→GPU.
 */

#include "odl_tb5_core.h"

#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/math.h>
#include <linux/vmalloc.h>

/* Maximum size of a single in-flight reassembled message (P0-1).  Matches the
 * sender-side cap in odl_tb5_stream_send(); a peer that streams continuation
 * frames with no MSG_END can otherwise grow the assembly buffer unbounded. */
#define ODL_TB5_RX_ASM_CAP_MAX ((size_t)ODL_TB5_STREAM_PAYLOAD_MAX * 4096)

/* Forward declarations for functions defined later in this file */
static void odl_tb5_stream_free(struct kref *ref);

/*
 * High-resolution fallback poll timer — kicks both TX and RX ring_work
 * every ODL_TB5_POLL_INTERVAL_NS while there is work to chase.
 *
 * NHI MSI-X interrupts DO fire, but ring_work triggered by the ISR
 * sometimes doesn't see completions yet (descriptor write-back delay).
 * This timer ensures completions are processed within one poll interval
 * instead of waiting for the next jiffy tick.  schedule_work is idempotent, so
 * ISR-driven and timer-driven kicks are safely additive.
 *
 * On-demand arming: the timer is not a free-running 100 kHz heartbeat.  It
 * runs only while TX frames are outstanding (tx_inflight > 0), a synchronous
 * RX dmabuf call is waiting, or RX frames arrived within the recent grace
 * window, and disarms otherwise so a connected-but-idle device costs zero
 * CPU.  odl_tb5_poll_kick() re-arms it on the next submit / RX arrival.
 * Because the ISR independently kicks
 * ring_work on real completions, disarming during idle only forgoes the
 * write-back re-check — never a completion.
 */

enum hrtimer_restart odl_tb5_rx_poll_timer_fn(struct hrtimer *timer)
{
	struct odl_tb5_device *dev =
		container_of(timer, struct odl_tb5_device, rx_poll_timer);

	bool any_started = false;
	bool keep;
	u64 rxseen;
	int i;

	if (atomic_read(&dev->removing)) {
		atomic_set(&dev->poll_active, 0);
		return HRTIMER_NORESTART;
	}

	for (i = 0; i < dev->num_paths; i++) {
		struct odl_tb5_path *path = &dev->paths[i];

		if (path->tx.ring && path->tx.started)
			schedule_work(&path->tx.ring->work);

		if (path->rx.ring && path->rx.started)
			schedule_work(&path->rx.ring->work);

		if (path->tx.started || path->rx.started)
			any_started = true;
	}

	/* Reset the grace counter whenever an RX frame was seen since the last
	 * tick; otherwise let it age.  These fields are touched only here, and
	 * the timer never runs concurrently with itself, so no locking. */
	rxseen = atomic64_read(&dev->stats.rx_frames_seen);
	if (rxseen != dev->poll_last_rxseen) {
		dev->poll_last_rxseen = rxseen;
		dev->poll_idle_ticks = 0;
	} else {
		dev->poll_idle_ticks++;
	}

	keep = atomic_read(&dev->tx_inflight) > 0 ||
	       atomic_read(&dev->rx_dmabuf_pending) > 0 ||
	       dev->poll_idle_ticks < ODL_TB5_POLL_GRACE_TICKS;

	if (dev->state >= ODL_TB5_STATE_CONNECTED && any_started && keep) {
		hrtimer_forward_now(timer,
				    ns_to_ktime(ODL_TB5_POLL_INTERVAL_NS));
		return HRTIMER_RESTART;
	}

	/*
	 * Going idle: publish poll_active = 0 so a future odl_tb5_poll_kick()
	 * will re-arm, then re-check for work that raced in between our
	 * "keep" evaluation and the store.  If one did, reclaim the armed flag
	 * (xchg) and keep polling; if poll_kick already reclaimed it we lose
	 * the race harmlessly (it did the hrtimer_start).  The ISR remains the
	 * correctness backstop regardless.
	 */
	atomic_set(&dev->poll_active, 0);
	if (dev->state >= ODL_TB5_STATE_CONNECTED && any_started &&
	    (atomic_read(&dev->tx_inflight) > 0 ||
	     atomic_read(&dev->rx_dmabuf_pending) > 0) &&
	    atomic_xchg(&dev->poll_active, 1) == 0) {
		hrtimer_forward_now(timer,
				    ns_to_ktime(ODL_TB5_POLL_INTERVAL_NS));
		return HRTIMER_RESTART;
	}

	return HRTIMER_NORESTART;
}

/*
 * (Re)arm the fallback poll if it is not already running.  Safe to call from
 * process/workqueue context on every TX submit and RX arrival; the xchg fast
 * path is a single atomic when the timer is already armed (the common case
 * under load).  hrtimer_start may run concurrently with the timer callback on
 * another CPU — the hrtimer core serialises this on the cpu_base lock.
 */
void odl_tb5_poll_kick(struct odl_tb5_device *dev)
{
	if (atomic_read(&dev->removing))
		return;
	if (dev->state < ODL_TB5_STATE_CONNECTED)
		return;
	if (atomic_xchg(&dev->poll_active, 1) == 0)
		hrtimer_start(&dev->rx_poll_timer,
			      ns_to_ktime(ODL_TB5_POLL_INTERVAL_NS),
			      HRTIMER_MODE_REL);
}

/* Cancel the fallback poll and clear the armed flag so a later kick re-arms
 * cleanly.  Callers that hold no timer-base lock only. */
void odl_tb5_poll_disarm(struct odl_tb5_device *dev)
{
	hrtimer_cancel(&dev->rx_poll_timer);
	atomic_set(&dev->poll_active, 0);
}

/* Bracket a successful tb_ring_tx: bump the outstanding-TX count and arm the
 * poll on the idle→busy edge.  Paired 1:1 with odl_tb5_tx_completed() in the
 * TX completion callbacks. */
void odl_tb5_tx_submitted(struct odl_tb5_device *dev)
{
	if (atomic_inc_return(&dev->tx_inflight) == 1)
		odl_tb5_poll_kick(dev);
}

/* Balance one successful TX submission.  Verification waits on the global
 * count before resetting the rings, so wake it when the last frame completes. */
static void odl_tb5_tx_completed(struct odl_tb5_device *dev)
{
	if (atomic_dec_and_test(&dev->tx_inflight))
		wake_up_all(&dev->verify_waitq);
}

/* One dma-buf transfer owns every frame in its stage array until WAIT has
 * observed all callbacks and releases the mapping. */
struct odl_tb5_dmabuf_stage {
	struct odl_tb5_frame_slot *slot;	/* NULL for raw-payload cells */
	void *cpu;
	dma_addr_t dma;		/* payload DMA address (raw: exporter page) */
	u32 seg_off;		/* chunk start offset within its sg */
	size_t offset;		/* dmabuf byte offset (framed copy only) */
	size_t len;		/* payload chunk */
	struct odl_tb5_dmabuf_xfer *xfer;
	u8 path;
	struct ring_frame frame; /* raw posts live here (no pool slot) */
};

static void odl_tb5_queue_ctrl_reply(struct odl_tb5_device *dev, int path_idx,
				    bool send_ack)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->verify_reply_lock, flags);
	if (dev->verify_reply_open && !atomic_read(&dev->removing)) {
		atomic_or(BIT(path_idx), send_ack ? &dev->verify_pong_mask :
						  &dev->verify_ping_mask);
		/* Queue under the admission lock so a later close + flush sees it. */
		schedule_work(&dev->ctrl_reply_work);
	}
	spin_unlock_irqrestore(&dev->verify_reply_lock, flags);
}

static void odl_tb5_record_pong(struct odl_tb5_device *dev, int path_idx)
{
	unsigned long flags;
	bool accepted = false;

	spin_lock_irqsave(&dev->verify_reply_lock, flags);
	if (dev->verify_reply_open && !atomic_read(&dev->removing)) {
		/* Admit the required ACK before publishing the PONG proof. */
		atomic_or(BIT(path_idx), &dev->verify_pong_mask);
		schedule_work(&dev->ctrl_reply_work);
		atomic_or(BIT(path_idx), &dev->pong_mask);
		accepted = true;
	}
	spin_unlock_irqrestore(&dev->verify_reply_lock, flags);
	if (accepted)
		wake_up_interruptible(&dev->verify_waitq);
}

static void odl_tb5_record_ack(struct odl_tb5_device *dev, int path_idx)
{
	unsigned long flags;
	bool accepted = false;

	spin_lock_irqsave(&dev->verify_reply_lock, flags);
	if (dev->verify_reply_open && !atomic_read(&dev->removing)) {
		atomic_or(BIT(path_idx), &dev->ack_mask);
		accepted = true;
	}
	spin_unlock_irqrestore(&dev->verify_reply_lock, flags);
	if (accepted)
		wake_up_interruptible(&dev->verify_waitq);
}

static void odl_tb5_record_drain_msg(struct odl_tb5_device *dev, int path_idx,
				     const struct odl_tb5_dma_hdr *hdr)
{
	u32 type = le32_to_cpu(hdr->type);
	u32 version = le32_to_cpu(hdr->reserved[0]);
	u32 value = le32_to_cpu(hdr->reserved[1]);
	u32 generation = le32_to_cpu(hdr->reserved[2]);
	unsigned long flags;
	bool accepted = false;

	if (version != ODL_TB5_DMA_DRAIN_VERSION) {
		pr_warn_ratelimited("OdinLink: unsupported DMA drain version %u\n",
				    version);
		return;
	}

	spin_lock_irqsave(&dev->verify_reply_lock, flags);
	if (!dev->drain_reply_open || atomic_read(&dev->removing))
		goto out;

	switch (type) {
	case ODL_TB5_DMA_DRAIN_PREP:
		if (!generation)
			break;
		atomic_set(&dev->drain_peer_generation[path_idx], generation);
		atomic_or(BIT(path_idx), &dev->drain_prep_mask);
		if (atomic_read(&dev->drain_frozen_mask) & BIT(path_idx)) {
			atomic_or(BIT(path_idx), &dev->drain_ready_reply_mask);
			schedule_work(&dev->ctrl_reply_work);
		}
		accepted = true;
		break;
	case ODL_TB5_DMA_DRAIN_READY:
		if (generation !=
		    (u32)atomic_read(&dev->drain_generation))
			break;
		atomic_set(&dev->drain_peer_rx[path_idx], value);
		atomic_or(BIT(path_idx), &dev->drain_ready_mask);
		accepted = true;
		break;
	case ODL_TB5_DMA_DRAIN_PAD:
		if (!generation || generation != (u32)atomic_read(
				&dev->drain_peer_generation[path_idx]))
			break;
		if (atomic_inc_return(&dev->drain_pad_received[path_idx]) ==
		    atomic_read(&dev->drain_pad_expected[path_idx])) {
			atomic_or(BIT(path_idx), &dev->drain_ack_reply_mask);
			schedule_work(&dev->ctrl_reply_work);
		}
		accepted = true;
		break;
	case ODL_TB5_DMA_DRAIN_ACK:
		if (generation !=
		    (u32)atomic_read(&dev->drain_generation))
			break;
		atomic_or(BIT(path_idx), &dev->drain_ack_mask);
		accepted = true;
		break;
	default:
		break;
	}
out:
	spin_unlock_irqrestore(&dev->verify_reply_lock, flags);
	if (accepted)
		wake_up_interruptible(&dev->verify_waitq);
}

/* Find which odl_tb5_device owns a given tb_ring and return its ring_ctx. */
static struct odl_tb5_ring_ctx *
odl_tb5_ring_to_ctx(struct tb_ring *ring)
{
	struct odl_tb5_device *dev;
	int i;

	list_for_each_entry_rcu(dev, &odl_tb5_devices_list, list) {
		for (i = 0; i < dev->num_paths; i++) {
			if (dev->paths[i].tx.ring == ring)
				return &dev->paths[i].tx;
			if (dev->paths[i].rx.ring == ring)
				return &dev->paths[i].rx;
		}
	}

	return NULL;
}

/* Given an RX tb_ring pointer, return the owning odl_tb5_device. */
struct odl_tb5_device *
odl_tb5_rx_ring_to_dev(struct tb_ring *ring)
{
	struct odl_tb5_device *dev;
	int i;

	list_for_each_entry_rcu(dev, &odl_tb5_devices_list, list) {
		for (i = 0; i < dev->num_paths; i++) {
			if (dev->paths[i].rx.ring == ring)
				return dev;
		}
	}

	return NULL;
}

void odl_tb5_tx_callback(struct tb_ring *ring,
			 struct ring_frame *frame, bool canceled)
{
	struct odl_tb5_ring_ctx *ctx;
	struct odl_tb5_frame_slot *slot;
	struct odl_tb5_tx_msg *msg;
	struct odl_tb5_device *dev;

	ctx = odl_tb5_ring_to_ctx(ring);
	if (WARN_ON_ONCE(!ctx))
		return;

	/* Check if this is a frame pool slot (new stream path) */
	dev = ctx->dev;

	if (atomic_read(&dev->removing))
		return;
	slot = container_of(frame, struct odl_tb5_frame_slot, frame);

	if (slot >= dev->frame_pool.slots &&
	    slot < dev->frame_pool.slots + dev->frame_pool.size) {
		ODL_STAT_INC(dev, tx_frames_completed);
		if (canceled)
			ODL_STAT_INC(dev, tx_frames_canceled);

		/* Paired with odl_tb5_tx_submitted() at the pool/ctrl TX
		 * submit sites; runs for canceled frames too. */
		odl_tb5_tx_completed(dev);

		msg = slot->tx_msg;
		odl_tb5_frame_pool_put(&dev->frame_pool, slot);

		if (msg) {
			if (atomic_dec_and_test(&msg->frames_pending) &&
			    msg->sent == msg->len) {
				struct odl_tb5_stream *s = msg->stream;

				atomic_inc(&s->tx_completed);
				atomic_dec(&s->tx_in_flight);
				wake_up_interruptible(&s->tx_waitq);
				kfree(msg);
			}
		}

		if (canceled)
			return;
	} else {
		/* Legacy path for proto layer direct ring submissions.
		 * Paired with odl_tb5_tx_submitted() at the legacy submit_tx /
		 * dmabuf-TX sites; decrement before the canceled early-return so
		 * canceled frames balance too. */
		odl_tb5_tx_completed(dev);

		if (canceled) {
			pr_debug("odl_tb5: TX callback canceled\n");
			return;
		}

		pr_debug("odl_tb5: TX complete frame=%px size=%u\n",
			 frame, frame->size);

		atomic_inc(&ctx->completed);
		wake_up_interruptible(&ctx->waitq);
	}
}

void odl_tb5_tx_batch_callback(struct tb_ring *ring,
			       struct ring_frame *frame, bool canceled)
{
	struct odl_tb5_ring_ctx *ctx;
	struct odl_tb5_device *dev;
	struct odl_tb5_batch_buf *batch = NULL;
	struct odl_tb5_tx_msg *msg;
	int b;

	ctx = odl_tb5_ring_to_ctx(ring);
	if (WARN_ON_ONCE(!ctx))
		return;

	dev = ctx->dev;

	if (atomic_read(&dev->removing))
		return;

	/* Identify which batch buffer owns this frame (8 entries max) */
	for (b = 0; b < ODL_TB5_BATCH_BUF_COUNT; b++) {
		struct odl_tb5_batch_buf *candidate = &dev->batch_pool.bufs[b];

		if (frame >= &candidate->frames[0] &&
		    frame < &candidate->frames[ODL_TB5_BATCH_FRAMES]) {
			batch = candidate;
			break;
		}
	}

	if (WARN_ON_ONCE(!batch))
		return;

	ODL_STAT_INC(dev, tx_frames_completed);
	if (canceled)
		ODL_STAT_INC(dev, tx_frames_canceled);

	/* Paired with odl_tb5_tx_submitted() in the throughput submit loop;
	 * one decrement per batch frame, canceled frames included. */
	odl_tb5_tx_completed(dev);

	msg = batch->tx_msg;

	/* Return batch buffer to pool when all its frames complete */
	if (atomic_dec_and_test(&batch->frames_pending))
		odl_tb5_batch_pool_put(&dev->batch_pool, batch);

	/* Complete the message when all batches are done */
	if (msg && atomic_dec_and_test(&msg->frames_pending) &&
	    msg->sent == msg->len) {
		struct odl_tb5_stream *s = msg->stream;

		atomic_inc(&s->tx_completed);
		atomic_dec(&s->tx_in_flight);
		wake_up_interruptible(&s->tx_waitq);
		kfree(msg);
	}
}

void odl_tb5_rx_callback(struct tb_ring *ring,
			 struct ring_frame *frame, bool canceled)
{
	struct odl_tb5_ring_ctx *ctx;
	struct odl_tb5_device *dev;
	struct odl_tb5_frame_slot *slot;
	int pidx;

	ctx = odl_tb5_ring_to_ctx(ring);
	if (WARN_ON_ONCE(!ctx))
		return;

	dev = odl_tb5_rx_ring_to_dev(ring);

	if (!dev || atomic_read(&dev->removing))
		return;

	/* Which path does this RX ring belong to?  The ctx is embedded in
	 * struct odl_tb5_path, so plain pointer arithmetic recovers the
	 * index — no extra list walk. */
	pidx = (int)(container_of(ctx, struct odl_tb5_path, rx) - dev->paths);
	if (WARN_ON_ONCE(pidx < 0 || pidx >= ODL_TB5_MAX_PATHS))
		return;

	/* Check if this is a frame pool slot (new stream path) */
	if (dev && dev->frame_pool.slots) {
		slot = container_of(frame, struct odl_tb5_frame_slot, frame);

		if (slot >= dev->frame_pool.slots &&
		    slot < dev->frame_pool.slots + dev->frame_pool.size) {
			void *data = slot->virt;

			ODL_STAT_INC(dev, rx_frames_seen);
			atomic64_inc(&dev->stats.path_rx_frames[pidx]);

			atomic_dec(&dev->paths[pidx].rx_posted);

			if (canceled) {
				ODL_STAT_INC(dev, rx_frames_canceled);
				atomic_inc(&dev->rx_canceled);
				odl_tb5_frame_pool_put(&dev->frame_pool, slot);
				return;
			}

			/* Real RX arrival — (re)arm the fallback poll so the
			 * write-back re-check covers frames that follow this
			 * burst even if the device was idle (timer disarmed). */
			odl_tb5_poll_kick(dev);

			/* The 12-bit size field cannot express 4096, so a
			 * completely filled frame wraps to 0.  Our TX caps
			 * frames at ODL_TB5_FRAME_LEN_MAX, but be defensive
			 * against peers that still send full frames: a real
			 * zero-length RX completion does not exist, so treat
			 * 0 as "max".  The stream header's payload_len
			 * governs the actual copy length. */
			if (frame->size == 0)
				frame->size = ODL_TB5_FRAME_LEN_MAX;
			/*
			 * The NHI tells us when a frame is bad. Until now
			 * nothing here looked. A CRC failure or a buffer
			 * overrun means the payload cannot be trusted, and
			 * accepting it hands corruption straight to the
			 * consumer — while dropping it without a counter is
			 * why missing frames have been impossible to
			 * attribute.
			 *
			 * Count, warn once in a while, and drop. Do not
			 * rate-limit away the only evidence: the counters are
			 * exact, only the printing is throttled.
			 */
			if (frame->flags & RING_DESC_CRC_ERROR) {
				atomic_inc(&dev->rx_err_crc);
				pr_warn_ratelimited("odl_tb5: RX CRC error (size=%u flags=0x%x) - frame dropped, total=%d\n",
						    frame->size, frame->flags,
						    atomic_read(&dev->rx_err_crc));
				odl_tb5_frame_pool_put(&dev->frame_pool, slot);
				return;
			}

			if (frame->flags & RING_DESC_BUFFER_OVERRUN) {
				atomic_inc(&dev->rx_err_overrun);
				pr_warn_ratelimited("odl_tb5: RX buffer overrun (size=%u flags=0x%x) - frame dropped, total=%d\n",
						    frame->size, frame->flags,
						    atomic_read(&dev->rx_err_overrun));
				odl_tb5_frame_pool_put(&dev->frame_pool, slot);
				return;
			}

			atomic_inc(&dev->rx_frames_ok);

			/* First check for raw DMA control message
			 * (no stream header — used during verify). */
			if (frame->size >= sizeof(struct odl_tb5_dma_hdr)) {
				__le32 raw_magic;

				memcpy(&raw_magic, data, sizeof(raw_magic));
				if (le32_to_cpu(raw_magic) == ODL_TB5_DMA_MAGIC) {
					struct odl_tb5_dma_hdr *dhdr = data;
					u32 type = le32_to_cpu(dhdr->type);

					ODL_STAT_INC(dev, rx_frames_ctrl);

					if (type == ODL_TB5_DMA_PONG) {
						pr_info("OdinLink: DMA pong received (pool)\n");
						odl_tb5_record_pong(dev, pidx);
					} else if (type == ODL_TB5_DMA_PING) {
						/* Remember the arrival path so
						 * the pong goes back on it. */
						odl_tb5_queue_ctrl_reply(dev, pidx, false);
					} else if (type == ODL_TB5_DMA_ACK) {
						odl_tb5_record_ack(dev, pidx);
					} else if (type >= ODL_TB5_DMA_DRAIN_PREP &&
						   type <= ODL_TB5_DMA_DRAIN_ACK) {
						odl_tb5_record_drain_msg(dev, pidx, dhdr);
					} else {
						pr_warn("OdinLink: unexpected DMA ctrl message type %d\n",
							type);
					}

					odl_tb5_frame_pool_put(&dev->frame_pool, slot);
					odl_tb5_rx_repost(dev, pidx);
					return;
				}
			}

			/* Completed with a full-size payload but the first dword was
			 * not the DMA magic, so it reached here via the stream
			 * check below.  Count it as unclassifiable — a control
			 * frame arriving in an unexpected layout, or stale data.
			 * Dump the first few so the exact contents are on record. */
			if (frame->size >= sizeof(struct odl_tb5_dma_hdr)) {
				atomic_inc(&dev->rx_unclassified);
				if (atomic_read(&dev->rx_unclassified) <= 3)
					pr_warn("odl_tb5: RX unclassified "
						"(size=%u flags=0x%x path=%d "
						"magic=%08x rx_ctrl=%lld "
						"ok=%d)\n",
						frame->size, frame->flags, pidx,
						le32_to_cpu(*(const __le32 *)data),
						(long long)atomic64_read(
							&dev->stats.rx_frames_ctrl),
						atomic_read(&dev->rx_frames_ok));
			}

			/* Check for stream header */
			if (frame->size >= ODL_TB5_STREAM_HDR_SIZE) {
				struct odl_tb5_stream_hdr *shdr = data;
				u8 dst_id = shdr->dst_id;

				if (dst_id != ODL_TB5_STREAM_ID_CTRL) {
					struct odl_tb5_stream *stream;
					u16 payload_len = le16_to_cpu(shdr->payload_len);

					ODL_STAT_INC(dev, rx_frames_stream);

					stream = odl_tb5_stream_lookup(dev, dst_id);
					if (stream && payload_len > 0) {
						const void *payload =
							data + ODL_TB5_STREAM_HDR_SIZE;
						u8 flags = shdr->flags;

						u16 fidx = le16_to_cpu(shdr->frag_idx);

						/* Start of new message — reset assembly */
						if (flags & ODL_TB5_SHDR_F_MSG_START) {
							ODL_STAT_INC(dev, rx_asm_start);
							if (stream->rx_asm_len > 0)
								ODL_STAT_INC(dev,
									rx_asm_reset_incomplete);
							kfree(stream->rx_asm_buf);
							stream->rx_asm_buf = NULL;
							stream->rx_asm_len = 0;
							stream->rx_asm_cap = 0;
							stream->rx_asm_src_id = shdr->src_id;
							stream->rx_asm_next_frag = 0;
							stream->rx_asm_bad = false;
						}

						/*
						 * Fragment sequencing: one compare, no
						 * round-trips, so the latency path is
						 * untouched (a SINGLE frame is always
						 * fidx 0 and trivially valid). Without
						 * this a dropped frame was invisible and
						 * silently produced a short, corrupt
						 * message. On a gap, poison the message
						 * and drop it at MSG_END rather than
						 * handing up bad data.
						 */
						if (fidx != stream->rx_asm_next_frag) {
							if (!stream->rx_asm_bad)
								/*
								 * Report the NHI error counters with the gap.
								 * If the missing frames were dropped by
								 * hardware, crc/ovr moved too and the cause is
								 * settled in one line. If every counter is
								 * zero, the frames were lost somewhere this
								 * driver can see, and the search moves inward.
								 */
								pr_warn_ratelimited("odl_tb5: stream %u fragment gap: got %u expected %u (lost %u) - dropping message; rx crc=%d ovr=%d cancel=%d short=%d len=%d ok=%d\n",
										    dst_id, fidx,
										    stream->rx_asm_next_frag,
										    fidx - stream->rx_asm_next_frag,
										    atomic_read(&dev->rx_err_crc),
										    atomic_read(&dev->rx_err_overrun),
										    atomic_read(&dev->rx_canceled),
										    atomic_read(&dev->rx_err_short),
										    atomic_read(&dev->rx_err_len),
										    atomic_read(&dev->rx_frames_ok));
							stream->rx_asm_bad = true;
							atomic_inc(&stream->rx_frag_drops);
						}
						stream->rx_asm_next_frag = fidx + 1;

						/* Append payload to assembly buffer.
						 * Cap the total size of one in-flight
						 * message to ODL_TB5_RX_ASM_CAP_MAX; a
						 * peer streaming continuation frames with
						 * no MSG_END would otherwise grow this
						 * unbounded and exhaust kernel memory
						 * (P0-1). */
						if (stream->rx_asm_len + payload_len >
						    stream->rx_asm_cap) {
							size_t need = stream->rx_asm_len +
								      payload_len;
							if (need > ODL_TB5_RX_ASM_CAP_MAX) {
								ODL_STAT_INC(dev,
									rx_asm_cap_exceeded);
								kfree(stream->rx_asm_buf);
								stream->rx_asm_buf = NULL;
								stream->rx_asm_len = 0;
								stream->rx_asm_cap = 0;
							} else {
								size_t new_cap = max_t(size_t,
									8192,
									max(stream->rx_asm_cap * 2,
									    need));
								void *nb = kmalloc(new_cap,
										   GFP_ATOMIC);
								if (nb) {
									if (stream->rx_asm_buf)
										memcpy(nb,
										       stream->rx_asm_buf,
										       stream->rx_asm_len);
									kfree(stream->rx_asm_buf);
									stream->rx_asm_buf = nb;
									stream->rx_asm_cap = new_cap;
								} else {
									ODL_STAT_INC(dev,
										rx_asm_grow_fail);
								}
							}
						}
						if (stream->rx_asm_buf &&
						    stream->rx_asm_len + payload_len <=
						    stream->rx_asm_cap) {
							memcpy(stream->rx_asm_buf +
							       stream->rx_asm_len,
							       payload, payload_len);
							stream->rx_asm_len += payload_len;
						} else {
							ODL_STAT_INC(dev,
								rx_asm_append_skipped);
							pr_warn_ratelimited("odl_tb5: rx_asm DROP payload_len=%u asm_len=%zu cap=%zu buf=%p\n",
									    payload_len,
									    stream->rx_asm_len,
									    stream->rx_asm_cap,
									    stream->rx_asm_buf);
						}

						/* End of message — enqueue complete msg */
						if (flags & ODL_TB5_SHDR_F_MSG_END) {
							struct odl_tb5_rx_msg *rxm;
							unsigned long rxflags;

							if (stream->rx_asm_bad) {
								/* Incomplete: never deliver it. */
								kfree(stream->rx_asm_buf);
								stream->rx_asm_buf = NULL;
								stream->rx_asm_len = 0;
								stream->rx_asm_cap = 0;
								stream->rx_asm_bad = false;
								stream->rx_asm_next_frag = 0;
								goto rx_frame_done;
							}

							rxm = kzalloc(sizeof(*rxm),
								      GFP_ATOMIC);
							if (rxm && stream->rx_asm_buf) {
								rxm->data = stream->rx_asm_buf;
								rxm->len = stream->rx_asm_len;
								rxm->src_id =
									stream->rx_asm_src_id;
								rxm->flags =
									ODL_TB5_SHDR_F_MSG_END;

								spin_lock_irqsave(
									&stream->rx_lock,
									rxflags);
								if (stream->rx_queue_len <
								    stream->rx_queue_max) {
									ODL_STAT_INC(dev,
										rx_msgs_enqueued);
									ODL_STAT_ADD(dev,
										rx_bytes_enqueued,
										rxm->len);
									list_add_tail(
										&rxm->list,
										&stream->rx_queue);
									stream->rx_queue_len++;
									spin_unlock_irqrestore(
										&stream->rx_lock,
										rxflags);
									atomic_inc(
										&stream->rx_complete);
									wake_up_interruptible(
										&stream->rx_waitq);
								} else {
									ODL_STAT_INC(dev,
										rx_msgs_drop_overflow);
									spin_unlock_irqrestore(
										&stream->rx_lock,
										rxflags);
									kfree(rxm->data);
									kfree(rxm);
								}

								/* Buffer ownership transferred
								 * to rx_msg (or freed above) */
								stream->rx_asm_buf = NULL;
								stream->rx_asm_len = 0;
								stream->rx_asm_cap = 0;
							} else {
								ODL_STAT_INC(dev,
									rx_msgs_drop_alloc);
								kfree(rxm);
								kfree(stream->rx_asm_buf);
								stream->rx_asm_buf = NULL;
								stream->rx_asm_len = 0;
								stream->rx_asm_cap = 0;
							}
						}

rx_frame_done:
						kref_put(&stream->refcount,
							 odl_tb5_stream_free);
					} else {
						/* stream_lookup took a ref even
						 * when payload_len == 0 */
						if (stream)
							kref_put(&stream->refcount,
								 odl_tb5_stream_free);
						ODL_STAT_INC(dev, rx_frames_no_stream);
					}
				}
			} else {
				ODL_STAT_INC(dev, rx_frames_runt);
			}

			odl_tb5_frame_pool_put(&dev->frame_pool, slot);
			odl_tb5_rx_repost(dev, pidx);
			return;
		}
	}

	/* Legacy path for proto layer direct ring submissions */
	ODL_STAT_INC(dev, rx_frames_legacy);
	atomic_dec(&dev->paths[pidx].legacy_rx_posted);

	if (canceled) {
		pr_debug("odl_tb5: RX callback canceled\n");
		return;
	}

	if (dev && dev->state >= ODL_TB5_STATE_CONNECTED) {
		int idx = frame - ctx->frames;
		void *data = ctx->bufs[ctx->posted_buf].virt +
			     ((size_t)idx * ODL_TB5_FRAME_SIZE);
		__le32 magic;

		memcpy(&magic, data, sizeof(magic));
		if (le32_to_cpu(magic) == ODL_TB5_DMA_MAGIC) {
			struct odl_tb5_dma_hdr *hdr = data;
			u32 type = le32_to_cpu(hdr->type);

			if (type == ODL_TB5_DMA_PONG) {
				pr_info("OdinLink: DMA pong received\n");
				odl_tb5_record_pong(dev, pidx);
			} else if (type == ODL_TB5_DMA_PING) {
				odl_tb5_queue_ctrl_reply(dev, pidx, false);
			} else if (type == ODL_TB5_DMA_ACK) {
				odl_tb5_record_ack(dev, pidx);
			} else if (type >= ODL_TB5_DMA_DRAIN_PREP &&
				   type <= ODL_TB5_DMA_DRAIN_ACK) {
				odl_tb5_record_drain_msg(dev, pidx, hdr);
			} else {
				pr_warn("OdinLink: unexpected DMA ctrl message type %d\n",
					type);
			}
			return;
		}
	}

	atomic_inc(&ctx->completed);
	wake_up_interruptible(&ctx->waitq);
}

/*
 * Completions for the synchronous dmabuf path (submit_tx_dmabuf /
 * submit_rx_dmabuf).  These frames live in path->{tx,rx}.frames[] and carry
 * synchronous dma-buf payload staged through the proven coherent frame pool.
 *
 * The shared odl_tb5_rx_callback's legacy branch peeks at
 * ctx->bufs[posted_buf].virt (the proto double-buffer) and treats a stale
 * ODL_TB5_DMA_MAGIC there as a ctrl PING/PONG, returning WITHOUT bumping
 * ctx->completed.  For dmabuf frames that buffer is unrelated to where the
 * data actually DMAed, so a coincidental magic match silently drops one
 * completion per transfer — the send/recv_dmabuf wait then never satisfies
 * (and, under E2E, the dropped RX also starves a credit and stalls one peer
 * TX frame).  A dedicated callback that only counts the completion avoids the
 * whole misinterpretation.
 */
void odl_tb5_tx_dmabuf_callback(struct tb_ring *ring,
				struct ring_frame *frame, bool canceled)
{
	struct odl_tb5_ring_ctx *ctx;
	struct odl_tb5_device *dev;

	ctx = odl_tb5_ring_to_ctx(ring);
	if (WARN_ON_ONCE(!ctx))
		return;

	dev = ctx->dev;
	if (!dev || atomic_read(&dev->removing))
		return;

	/* Paired with odl_tb5_tx_submitted() in submit_tx_dmabuf; runs for
	 * canceled frames too so tx_inflight stays balanced. */
	odl_tb5_tx_completed(dev);

	if (canceled)
		return;

	atomic_inc(&ctx->completed);
	wake_up_interruptible(&ctx->waitq);
}

void odl_tb5_rx_dmabuf_callback(struct tb_ring *ring,
				struct ring_frame *frame, bool canceled)
{
	struct odl_tb5_ring_ctx *ctx;
	struct odl_tb5_device *dev;
	struct odl_tb5_frame_slot *slot;
	struct odl_tb5_dmabuf_xfer *x;

	ctx = odl_tb5_ring_to_ctx(ring);
	if (WARN_ON_ONCE(!ctx))
		return;

	dev = ctx->dev;
	if (!dev || atomic_read(&dev->removing))
		return;

	if (canceled)
		return;

	/* Framed dma-buf RX uses a pool slot, but unlike the shared stream
	 * callback the slot stays owned by the parked transfer until WAIT.
	 * Charge the callback to that transfer before publishing the ring-wide
	 * completion. */
	slot = container_of(frame, struct odl_tb5_frame_slot, frame);
	x = READ_ONCE(slot->dmabuf_xfer);
	if (!WARN_ON_ONCE(!x || slot->dmabuf_path >= ODL_TB5_MAX_PATHS))
		atomic_inc(&x->done[slot->dmabuf_path]);

	/* Count this arrival as RX activity so the poll timer's idle-grace
	 * resets, and re-arm it if it had disarmed — keeps the completion pump
	 * running for the remaining posted frames of this recv (see
	 * submit_rx_dmabuf). */
	ODL_STAT_INC(dev, rx_frames_seen);
	odl_tb5_poll_kick(dev);

	atomic_inc(&ctx->completed);
	wake_up_interruptible(&ctx->waitq);
}

void odl_tb5_rx_dmabuf_raw_callback(struct tb_ring *ring,
				    struct ring_frame *frame, bool canceled)
{
	struct odl_tb5_ring_ctx *ctx;
	struct odl_tb5_device *dev;
	struct odl_tb5_dmabuf_stage *stage;
	struct odl_tb5_dmabuf_xfer *x;

	ctx = odl_tb5_ring_to_ctx(ring);
	if (WARN_ON_ONCE(!ctx))
		return;

	dev = ctx->dev;
	if (!dev || atomic_read(&dev->removing))
		return;

	if (canceled)
		return;

	stage = container_of(frame, struct odl_tb5_dmabuf_stage, frame);
	x = READ_ONCE(stage->xfer);
	if (WARN_ON_ONCE(!x || stage->path >= ODL_TB5_MAX_PATHS))
		x = NULL;

	/* The NHI writes the received byte count into the completed
	 * descriptor; charge it to the cumulative raw-RX counter so the
	 * submit-side length validation can compare the transfer's delta. */
	if (frame->size)
		atomic_add(frame->size, &ctx->rx_raw_bytes);

	ODL_STAT_INC(dev, rx_frames_seen);
	odl_tb5_poll_kick(dev);
	/* Publish transfer completion only after the callback's final access to
	 * the private stage/frame.  A timeout=0 poll may release it as soon as
	 * this counter changes. */
	if (x)
		atomic_inc(&x->done[stage->path]);

	atomic_inc(&ctx->completed);
	wake_up_interruptible(&ctx->waitq);
}

/* Allocate the TX/RX ring pair + output HopID for one path. */
static int odl_tb5_ring_pair_alloc(struct odl_tb5_device *dev, int idx,
				   unsigned int rs)
{
	struct tb_xdomain *xd = dev->xd;
	struct odl_tb5_path *path = &dev->paths[idx];
	unsigned int sof_mask, eof_mask;
	unsigned int ring_flags;
	int ret;

	path->tx.ring_size = rs;
	path->rx.ring_size = rs;

	path->tx.frames = kvzalloc(rs * sizeof(struct ring_frame), GFP_KERNEL);
	if (!path->tx.frames)
		return -ENOMEM;

	path->rx.frames = kvzalloc(rs * sizeof(struct ring_frame), GFP_KERNEL);
	if (!path->rx.frames) {
		ret = -ENOMEM;
		goto err_free_tx_frames;
	}

	ret = tb_xdomain_alloc_out_hopid(xd, -1);
	if (ret < 0) {
		if (ret == -ENOSPC) {
			pr_err("odl_tb5: no output HopID available; another driver "
				"may own the controller resources (for example "
				"thunderbolt_ibverbs)\n");
			/* Preserve a distinct, internal signal for the probe retry
			 * policy; a smaller DMA buffer cannot free a HopID. */
			ret = -EBUSY;
		} else {
			pr_err("odl_tb5: failed to allocate output HopID: %d\n",
				ret);
		}
		goto err_free_rx_frames;
	}
	path->local_tx_hopid = ret;

	ring_flags = RING_FLAG_FRAME;
	if (odl_e2e)
		ring_flags |= RING_FLAG_E2E;

	path->tx.ring = tb_ring_alloc_tx(xd->tb->nhi, -1,
					rs,
					ring_flags);
	if (!path->tx.ring) {
		/* The ring API returns only NULL and does not expose whether this
		 * was memory, IRQ, or slot allocation.  Keep the diagnosis neutral
		 * and allow the size ladder to handle memory pressure. */
		pr_err("odl_tb5: failed to allocate NHI TX ring\n");
		ret = -ENOMEM;
		goto err_free_hopid;
	}

	sof_mask = BIT(ODL_TB5_PDF_SOF_DATA);
	/* EOF_RAW (bit 3) is reserved for frame classification but is never
	 * placed on the wire: raw-payload frames transmit with the standard
	 * EOF_DATA marker (an eof=3 descriptor made the NHI drop the E2E
	 * credit-return for the first posted frame).  Keep the mask on
	 * EOF_DATA only; legacy peers never see anything else. */
	eof_mask = BIT(ODL_TB5_PDF_EOF_DATA);

	/* E2E credits pair each RX ring with its own path's TX ring —
	 * both sides allocate symmetrically (known constraint). */
	path->rx.ring = tb_ring_alloc_rx(xd->tb->nhi, -1,
					rs,
					ring_flags,
					path->tx.ring->hop,
					sof_mask, eof_mask,
					NULL, NULL);
	if (!path->rx.ring) {
		pr_err("odl_tb5: failed to allocate NHI RX ring\n");
		ret = -ENOMEM;
		goto err_free_tx_ring;
	}

	/* Ensure the NHI DMA device advertises a 64-bit streaming mask.  All
	 * our dmabuf/frame-pool mappings (dma_buf_map_attachment, dma_map_sg)
	 * go through tb_ring_dma_device(); if that device has no streaming
	 * dma_mask, dma_map_sg WARNs and, with an IOMMU active, can hand back
	 * wrong addresses.  Today amd_iommu=off masks this (direct mapping),
	 * but setting the mask makes the path correct with the IOMMU on too.
	 * The NHI device is shared across paths, so this is idempotent. */
	{
		struct device *dma_dev = tb_ring_dma_device(path->tx.ring);

		if (dma_dev && dma_set_mask_and_coherent(dma_dev,
							 DMA_BIT_MASK(64)))
			pr_warn("odl_tb5: 64-bit DMA mask unavailable on NHI "
				"device; leaving existing mask\n");
	}

	pr_info("odl_tb5: rings allocated: TX hop=%d, RX hop=%d, "
		"local_tx_hopid=%d (E2E=%s, e2e_tx_hop=%d)\n",
		path->tx.ring->hop, path->rx.ring->hop,
		path->local_tx_hopid, odl_e2e ? "enabled" : "disabled",
		odl_e2e ? path->tx.ring->hop : -1);

	path->tx.dev = dev;
	path->rx.dev = dev;
	spin_lock_init(&path->tx.lock);
	spin_lock_init(&path->rx.lock);
	atomic_set(&path->tx.completed, 0);
	atomic_set(&path->tx.submitted, 0);
	atomic_set(&path->rx.completed, 0);
	atomic_set(&path->rx.submitted, 0);
	atomic_set(&path->rx_reposting, 0);
	path->rx_repost_pending = false;
	init_waitqueue_head(&path->tx.waitq);
	init_waitqueue_head(&path->rx.waitq);
	init_waitqueue_head(&path->rx_repost_waitq);

	return 0;

err_free_tx_ring:
	tb_ring_free(path->tx.ring);
	path->tx.ring = NULL;
err_free_hopid:
	tb_xdomain_release_out_hopid(xd, path->local_tx_hopid);
	path->local_tx_hopid = -1;
err_free_rx_frames:
	kvfree(path->rx.frames);
	path->rx.frames = NULL;
err_free_tx_frames:
	kvfree(path->tx.frames);
	path->tx.frames = NULL;
	return ret;
}

int odl_tb5_rings_alloc(struct odl_tb5_device *dev, unsigned int rs)
{
	int ret, i;

	if (!rs)
		rs = odl_ring_size;
	if (rs < ODL_TB5_RING_SIZE_MIN)
		rs = ODL_TB5_RING_SIZE_MIN;
	if (rs > ODL_TB5_RING_SIZE_MAX)
		rs = ODL_TB5_RING_SIZE_MAX;
	rs = roundup_pow_of_two(rs);

	pr_info("odl_tb5: ring_size=%u (%u MB per batch, %u MB total)\n",
		rs,
		(rs * ODL_TB5_FRAME_SIZE) >> 20,
		(rs * ODL_TB5_FRAME_SIZE * ODL_TB5_NUM_BUFFERS * 2) >> 20);

	for (i = 0; i < dev->num_paths; i++) {
		ret = odl_tb5_ring_pair_alloc(dev, i, rs);
		if (ret) {
			if (i == 0)
				return ret;
			/* NHI ring scarcity (e.g. tbnet is loaded): keep
			 * whatever we already have and stripe less. */
			pr_warn("odl_tb5: path %d ring alloc failed (%d), "
				"continuing with %d path(s)\n", i, ret, i);
			dev->num_paths = i;
			break;
		}
	}

	return 0;
}

void odl_tb5_rings_free(struct odl_tb5_device *dev)
{
	int i;

	for (i = 0; i < ODL_TB5_MAX_PATHS; i++) {
		struct odl_tb5_path *path = &dev->paths[i];

		if (path->rx.ring) {
			tb_ring_free(path->rx.ring);
			path->rx.ring = NULL;
		}

		if (path->tx.ring) {
			tb_ring_free(path->tx.ring);
			path->tx.ring = NULL;
		}

		if (path->local_tx_hopid >= 0) {
			tb_xdomain_release_out_hopid(dev->xd,
						     path->local_tx_hopid);
			path->local_tx_hopid = -1;
		}

		kvfree(path->tx.frames);
		path->tx.frames = NULL;
		kvfree(path->rx.frames);
		path->rx.frames = NULL;
	}
}

int odl_tb5_rings_start(struct odl_tb5_device *dev, int idx)
{
	tb_ring_start(dev->paths[idx].tx.ring);
	tb_ring_start(dev->paths[idx].rx.ring);
	dev->paths[idx].tx.started = true;
	dev->paths[idx].rx.started = true;
	return 0;
}

void odl_tb5_rings_stop_path(struct odl_tb5_device *dev, int idx)
{
	struct odl_tb5_path *path = &dev->paths[idx];

	/* teardown UAF fix (B): the invariant enforced here is that a
	 * ring_frame is freed/unmapped ONLY after the thunderbolt ring can no
	 * longer reference it.  tb_ring_stop() cancels every in-flight frame
	 * (ring_work runs its callback with canceled=true, unlinking it from
	 * ring->queue) before returning.  For the dmabuf path, any frame that
	 * was still queued at drain-timeout was already cancelled by the
	 * stop+start in odl_tb5_submit_dmabuf's timeout/error path, so these
	 * rings are empty and tb_ring_stop's flush_work completes promptly
	 * instead of wedging rmmod in D state.  Pool frames are static-only
	 * freed later in odl_tb5_frame_pool_free(), which runs AFTER this stop
	 * in odl_tb5_remove, so no frame is freed while TB still owns it. */
	if (path->tx.ring && path->tx.started) {
		tb_ring_stop(path->tx.ring);
		path->tx.started = false;
		path->tx.frames_posted = false;
	}

	if (path->rx.ring && path->rx.started) {
		tb_ring_stop(path->rx.ring);
		path->rx.started = false;
		path->rx.frames_posted = false;
	}
}

void odl_tb5_rings_stop(struct odl_tb5_device *dev)
{
	int i;

	for (i = 0; i < ODL_TB5_MAX_PATHS; i++)
		odl_tb5_rings_stop_path(dev, i);
}

/* Reset all started rings to a clean state after kernel verification. */
void odl_tb5_rings_reset(struct odl_tb5_device *dev)
{
	int i;

	for (i = 0; i < ODL_TB5_MAX_PATHS; i++) {
		struct odl_tb5_path *path = &dev->paths[i];

		if (path->tx.ring && path->tx.started) {
			tb_ring_stop(path->tx.ring);
			tb_ring_start(path->tx.ring);
			path->tx.frames_posted = false;
			path->tx.swapped_since_post = false;
			atomic_set(&path->tx.completed, 0);
			atomic_set(&path->tx.submitted, 0);
		}

		if (path->rx.ring && path->rx.started) {
			tb_ring_stop(path->rx.ring);
			tb_ring_start(path->rx.ring);
			path->rx.frames_posted = false;
			path->rx.swapped_since_post = false;
			atomic_set(&path->rx.completed, 0);
			atomic_set(&path->rx.submitted, 0);
		}
	}
}

int odl_tb5_dma_bufs_alloc(struct odl_tb5_device *dev)
{
	struct device *dma_dev;
	size_t buf_size;
	int i;

	dma_dev = tb_ring_dma_device(dev->paths[0].tx.ring);
	buf_size = (size_t)ODL_TB5_FRAME_SIZE * dev->paths[0].tx.ring_size;

	for (i = 0; i < ODL_TB5_NUM_BUFFERS; i++) {
		dev->paths[0].tx.bufs[i].size = buf_size;
		dev->paths[0].tx.bufs[i].virt = dma_alloc_coherent(dma_dev, buf_size,
							  &dev->paths[0].tx.bufs[i].phys,
							  GFP_KERNEL);
		if (!dev->paths[0].tx.bufs[i].virt) {
			pr_err("odl_tb5: failed to alloc TX DMA buf %d (%zu bytes)\n",
			       i, buf_size);
			goto err_free;
		}

		dev->paths[0].rx.bufs[i].size = buf_size;
		dev->paths[0].rx.bufs[i].virt = dma_alloc_coherent(dma_dev, buf_size,
							  &dev->paths[0].rx.bufs[i].phys,
							  GFP_KERNEL);
		if (!dev->paths[0].rx.bufs[i].virt) {
			pr_err("odl_tb5: failed to alloc RX DMA buf %d (%zu bytes)\n",
			       i, buf_size);
			goto err_free;
		}
	}

	dev->paths[0].tx.front = 0;
	dev->paths[0].tx.back  = 1;
	dev->paths[0].rx.front = 0;
	dev->paths[0].rx.back  = 1;
	dev->paths[0].tx.frames_posted = false;
	dev->paths[0].tx.swapped_since_post = false;
	dev->paths[0].rx.frames_posted = false;
	dev->paths[0].rx.swapped_since_post = false;

	return 0;

err_free:
	odl_tb5_dma_bufs_free(dev);
	return -ENOMEM;
}

void odl_tb5_dma_bufs_free(struct odl_tb5_device *dev)
{
	struct device *dma_dev;
	int i;

	if (dev->paths[0].tx.ring)
		dma_dev = tb_ring_dma_device(dev->paths[0].tx.ring);
	else if (dev->paths[0].rx.ring)
		dma_dev = tb_ring_dma_device(dev->paths[0].rx.ring);
	else
		return;

	for (i = 0; i < ODL_TB5_NUM_BUFFERS; i++) {
		if (dev->paths[0].tx.bufs[i].virt) {
			dma_free_coherent(dma_dev,
					  dev->paths[0].tx.bufs[i].size,
					  dev->paths[0].tx.bufs[i].virt,
					  dev->paths[0].tx.bufs[i].phys);
			dev->paths[0].tx.bufs[i].virt = NULL;
		}

		if (dev->paths[0].rx.bufs[i].virt) {
			dma_free_coherent(dma_dev,
					  dev->paths[0].rx.bufs[i].size,
					  dev->paths[0].rx.bufs[i].virt,
					  dev->paths[0].rx.bufs[i].phys);
			dev->paths[0].rx.bufs[i].virt = NULL;
		}
	}
}

int odl_tb5_submit_tx(struct odl_tb5_device *dev,
		      size_t offset, size_t len, bool ctrl)
{
	struct odl_tb5_dma_buf *buf;
	size_t remaining;
	int nframes, i, ret;

	if (dev->state != ODL_TB5_STATE_CONNECTED &&
	    dev->state != ODL_TB5_STATE_READY &&
	    !dev->paths[0].tx.started)
		return -ENOTCONN;

	if (dev->paths[0].tx.frames_posted) {
		int sub = atomic_read(&dev->paths[0].tx.submitted);
		long tw = wait_event_interruptible_timeout(dev->paths[0].tx.waitq,
				atomic_read(&dev->paths[0].tx.completed) >= sub,
				msecs_to_jiffies(5000));
		if (tw <= 0) {
			pr_warn("odl_tb5: TX drain timeout (%ld), "
				"resetting ring\n", tw);
			tb_ring_stop(dev->paths[0].tx.ring);
			tb_ring_start(dev->paths[0].tx.ring);
		}
		dev->paths[0].tx.frames_posted = false;
		dev->paths[0].tx.swapped_since_post = false;
		atomic_set(&dev->paths[0].tx.completed, 0);
		atomic_set(&dev->paths[0].tx.submitted, 0);
	}

	buf = &dev->paths[0].tx.bufs[dev->paths[0].tx.front];

	if (offset > buf->size || len > buf->size - offset)
		return -EINVAL;

	nframes = DIV_ROUND_UP(len, ODL_TB5_FRAME_SIZE);
	remaining = len;

	for (i = 0; i < nframes; i++) {
		struct ring_frame *frame = &dev->paths[0].tx.frames[i];

		frame->buffer_phy = buf->phys + offset +
				    ((size_t)i * ODL_TB5_FRAME_SIZE);
		/* NOTE: full 4096-byte slices wrap the 12-bit size field to 0
		 * and transmit empty (legacy path; only used for small ctrl
		 * messages today — kept unchanged, see ODL_TB5_FRAME_LEN_MAX) */
		frame->size = min_t(size_t, ODL_TB5_FRAME_SIZE, remaining);
		frame->callback = odl_tb5_tx_callback;
		frame->sof = ctrl ? ODL_TB5_PDF_SOF_CTRL : ODL_TB5_PDF_SOF_DATA;
		frame->eof = ctrl ? ODL_TB5_PDF_EOF_CTRL : ODL_TB5_PDF_EOF_DATA;

		ret = tb_ring_tx(dev->paths[0].tx.ring, frame);
		if (ret < 0) {
			pr_err("odl_tb5: tb_ring_tx failed at frame %d: %d\n",
			       i, ret);
			return ret;
		}
		odl_tb5_tx_submitted(dev);

		remaining -= frame->size;
	}

	atomic_add(nframes, &dev->paths[0].tx.submitted);
	dev->paths[0].tx.frames_posted = true;

	pr_debug("odl_tb5: TX submitted %d frames, offset=%zu len=%zu ctrl=%d "
		"buf_phys=%pad ring_hop=%d\n",
		nframes, offset, len, ctrl,
		&buf->phys, dev->paths[0].tx.ring->hop);

	return 0;
}

int odl_tb5_submit_rx(struct odl_tb5_device *dev,
		      size_t offset, size_t len)
{
	struct odl_tb5_dma_buf *buf;
	size_t remaining;
	int nframes, i, ret;

	if (dev->state != ODL_TB5_STATE_CONNECTED &&
	    dev->state != ODL_TB5_STATE_READY &&
	    !dev->paths[0].rx.started)
		return -ENOTCONN;

	if (dev->paths[0].rx.frames_posted) {
		if (!dev->paths[0].rx.swapped_since_post)
			return 0;

		tb_ring_stop(dev->paths[0].rx.ring);
		tb_ring_start(dev->paths[0].rx.ring);
		dev->paths[0].rx.frames_posted = false;
		dev->paths[0].rx.swapped_since_post = false;
		atomic_set(&dev->paths[0].rx.completed, 0);
		atomic_set(&dev->paths[0].rx.submitted, 0);
	}

	buf = &dev->paths[0].rx.bufs[dev->paths[0].rx.front];

	if (offset > buf->size || len > buf->size - offset)
		return -EINVAL;

	nframes = DIV_ROUND_UP(len, ODL_TB5_FRAME_SIZE);
	remaining = len;

	for (i = 0; i < nframes; i++) {
		struct ring_frame *frame = &dev->paths[0].rx.frames[i];

		frame->buffer_phy = buf->phys + offset +
				    ((size_t)i * ODL_TB5_FRAME_SIZE);
		frame->size = min_t(size_t, ODL_TB5_FRAME_SIZE, remaining);
		frame->callback = odl_tb5_rx_callback;
		frame->sof = ODL_TB5_PDF_SOF_DATA;
		frame->eof = ODL_TB5_PDF_EOF_DATA;

		ret = tb_ring_rx(dev->paths[0].rx.ring, frame);
		if (ret < 0) {
			pr_err("odl_tb5: tb_ring_rx failed at frame %d: %d\n",
			       i, ret);
			return ret;
		}
		atomic_inc(&dev->paths[0].legacy_rx_posted);

		remaining -= frame->size;
	}

	atomic_add(nframes, &dev->paths[0].rx.submitted);
	dev->paths[0].rx.frames_posted = true;
	dev->paths[0].rx.posted_buf = dev->paths[0].rx.front;

	pr_debug("odl_tb5: RX submitted %d frames, offset=%zu len=%zu "
		"buf_phys=%pad ring_hop=%d\n",
		nframes, offset, len,
		&buf->phys, dev->paths[0].rx.ring->hop);

	return 0;
}

/*
 * Multi-path striped dmabuf transfer (shared TX/RX core).
 *
 * A single transfer is copied through coherent pool slots, chunked to leave
 * room for a private frame header, and
 * BLOCK-striped across the active paths: path p owns the contiguous byte range
 * [p*len/nps, (p+1)*len/nps).  Each path therefore writes one contiguous region
 * of the destination rather than every other frame — frame-interleaved striping
 * has two DMA engines writing physically adjacent, cache-line-sharing memory at
 * once, which corrupts frame-boundary bytes once the link is driven at full
 * 2-path rate (single-path is clean).  Both peers run this same code with the
 * same nps (dev->negotiated_paths — the symmetric quantity agreed at login,
 * min of each side's num_paths) and chunk identically, so a given frame maps to
 * the same path on both ends; E2E credits pair path p's TX ring with the peer's
 * path p RX ring, so it lands at the matching dmabuf offset.  Each frame carries
 * its own absolute buffer_phy, so cross-path arrival ordering is irrelevant;
 * only same-path FIFO order matters, and per path we post in ascending offset.
 *
 * TX completion accounting uses each ring context.  RX additionally charges
 * every callback to its owning transfer: several NOWAIT receives may share a
 * ring, so a ring-wide count cannot say which parked token completed.
 *
 * nps == 1 collapses to the original single-path behaviour (everything on
 * paths[0]).
 */

/* Drain a ring that still has in-flight frames before the DMA mapping is
 * dropped (teardown UAF (B)), and after a signal interrupted the submit.
 * RX must wait on @owned: unrelated NOWAIT transfers share the ring-wide
 * counter and may complete later descriptors while this transfer is short.
 * TX is synchronous/serialized and continues to use its ring FIFO range.
 * Re-baseline the counters and keep the ring running so the next submit
 * stays compatible.  stop+start is the last resort: it resets the
 * driver-side head to 0 but the NHI controller's internal descriptor
 * position survives, so a re-started ring makes the DMA engine resume from
 * a stale slot -- new posts behind the engine's head never complete and
 * stale descriptors in its path "complete" without advancing (peer sees
 * rx_frames_seen == 0, the F2 hang).  Only a genuinely wedged ring (drain
 * timeout) gets the stop+start cancellation.
 *
 * Returns true if the ring drained and is still running, false if it had
 * to be stop+started. */
static bool odl_tb5_dmabuf_reclaim(struct odl_tb5_ring_ctx *rc,
				       long baseline, long need,
				       atomic_t *owned,
				       bool unpublished)
{
	bool drained;

	if (!rc->ring || !rc->started)
		return true;

	/* A receive mapping is safe to drop only after this transfer's private
	 * frames called back.  Ring FIFO progress alone is not ownership proof
	 * when several NOWAIT tokens are parked concurrently. */
	drained = wait_event_timeout(rc->waitq,
		owned ? atomic_read(owned) >= need :
			atomic_read(&rc->completed) >= baseline + need,
		msecs_to_jiffies(5000));
	if (drained) {
		/* Partial-submit error frames were posted but never published in
		 * submitted; account for exactly those descriptors.  A parked
		 * NOWAIT transfer was already published, so rebasing it to the
		 * current completion count would move the reservation tail behind
		 * later tokens which are still in flight. */
		if (unpublished)
			atomic_add(need, &rc->submitted);
		return true;
	}

	pr_warn("odl_tb5: ring drain timeout (completed=%d submitted=%d "
		"owned=%d baseline=%ld need=%ld tx_inflight=%d poll_active=%d "
		"poll_idle_ticks=%u e2e=%d); canceling frames via ring stop\n",
		atomic_read(&rc->completed), atomic_read(&rc->submitted),
		owned ? atomic_read(owned) : -1,
		baseline, need,
		atomic_read(&rc->dev->tx_inflight),
		atomic_read(&rc->dev->poll_active),
		rc->dev->poll_idle_ticks,
		odl_e2e ? 1 : 0);
	tb_ring_stop(rc->ring);
	tb_ring_start(rc->ring);
	atomic_set(&rc->completed, 0);
	atomic_set(&rc->submitted, 0);
	return false;
}

/* Keep the dry-run capacity check and the actual submit on exactly the same
 * block-stripe rule.  Path 0 is the single-path fallback; otherwise payload
 * paths are numbered 1..ndp. */
static int odl_tb5_dmabuf_path(int ndp, size_t len, size_t sent)
{
	int dp;

	if (ndp == 0)
		return 0;
	dp = (int)((u64)sent * ndp / len);
	if (dp >= ndp)
		dp = ndp - 1;
	return 1 + dp;
}

/* Mirror the real chunking of the submit loop over the mapped SG layout:
 * block-stripe path choice from the byte position, cells cut at sg
 * boundaries with the given per-cell cap.  Fills the per-path descriptor
 * counts (caller zeroes @needed first) and enforces the per-path ring
 * occupancy limit like the submit loop does.  @raw_geom, when non-NULL,
 * is cleared if any cell except the transfer tail is shorter than
 * @chunk_max — the raw-payload gate: bounce-free cells must be full-size
 * everywhere but the tail, or the two importers' chunkings can differ
 * (64-byte residue syndromes).  Returns 0, or -EIO if the SG addresses do
 * not cover offset+len (matches the errno the framed preflight used). */
static int odl_tb5_dmabuf_walk(struct sg_table *sgt, size_t offset,
			       size_t len, int ndp, bool is_tx,
			       struct odl_tb5_device *dev,
			       int rx_pool_posted, bool rx_shared,
			       size_t chunk_max, int *needed, bool *raw_geom)
{
	struct scatterlist *sg;
	size_t skip = offset;
	size_t total_remaining = len;
	int nents_i;

	for_each_sgtable_dma_sg(sgt, sg, nents_i) {
		size_t count_remaining = sg_dma_len(sg);

		if (skip > 0) {
			if ((size_t)skip >= count_remaining) {
				skip -= count_remaining;
				continue;
			}
			count_remaining -= skip;
			skip = 0;
		}

		while (count_remaining > 0 && total_remaining > 0) {
			struct odl_tb5_ring_ctx *rc;
			size_t sent = len - total_remaining;
			size_t chunk;
			int occupied;
			int p = odl_tb5_dmabuf_path(ndp, len, sent);

			rc = is_tx ? &dev->paths[p].tx : &dev->paths[p].rx;
			occupied = (!is_tx && rx_shared && p == 0) ?
				   rx_pool_posted : 0;
			if (needed[p] >= rc->ring_size - occupied)
				return -ENOSPC;
			chunk = min3(chunk_max, count_remaining,
				     total_remaining);
			if (raw_geom && *raw_geom && chunk < chunk_max &&
			    chunk < total_remaining)
				*raw_geom = false;
			needed[p]++;
			count_remaining -= chunk;
			total_remaining -= chunk;
		}
		if (total_remaining == 0)
			break;
	}
	return total_remaining == 0 ? 0 : -EIO;
}

/* Release a parked transfer's mapping, staged frames and pool slots.
 * @deliver: copy received payload back (success path).  The raw-payload
 * length validation runs here — with no in-band header the received byte
 * sum is the only integrity check, and any shortfall fails the transfer
 * (the framed path cannot hit this: the staged copy is exact by
 * construction).  Returns -EIO when the validation fails. */
static int odl_tb5_dmabuf_release(struct odl_tb5_device *dev,
				  struct odl_tb5_dmabuf_xfer *x, bool deliver)
{
	int p;

	if (deliver && x->raw_ok && !x->is_tx) {
		size_t got = 0;

		/* Each raw RX stage owns the exact ring_frame posted for this
		 * transfer, and the NHI writes that descriptor's received length
		 * before invoking its callback.  Summing the transfer's frames is
		 * concurrency-safe; subtracting a device-wide byte-counter baseline
		 * also counted later NOWAIT receives that completed first. */
		smp_rmb(); /* completion count was observed before frame lengths */
		for (p = 0; p < x->stage_count; p++)
			if (!x->stage[p].slot)
				got += x->stage[p].frame.size;
		if (got != x->len) {
			pr_warn_ratelimited("odl_tb5: raw RX length mismatch "
				"(got %zu want %zu)\n", got, x->len);
			ODL_STAT_INC(dev, raw_rx_len_mismatch);
			deliver = false;   /* don't copy back unvalidated data */
		}
	}
	if (x->rx_armed) {
		atomic_dec(&dev->dmabuf_rx_active);
		x->rx_armed = false;
	}
	if (x->rx_poll_held) {
		atomic_dec(&dev->rx_dmabuf_pending);
		x->rx_poll_held = false;
	}
	/* Raw stages hold no pool slot (the NHI wrote the exporter pages
	 * directly); only framed stages are copied back and returned. */
	for (p = 0; p < x->stage_count; p++) {
		if (!x->stage[p].slot)
			continue;
		if (deliver && !x->is_tx)
			iosys_map_memcpy_to(&x->cpu_map, x->stage[p].offset,
					    x->stage[p].cpu +
						ODL_TB5_STREAM_HDR_SIZE,
					    x->stage[p].len);
		odl_tb5_frame_pool_put(&dev->frame_pool, x->stage[p].slot);
		x->stage[p].slot = NULL;
	}
	if (x->cpu_mapped)
		dma_buf_vunmap(x->dmabuf, &x->cpu_map);
	if (x->cpu_access)
		dma_buf_end_cpu_access(x->dmabuf, DMA_BIDIRECTIONAL);
	kfree(x->stage);
	x->stage = NULL;
	if (x->mapped) {
		dma_sync_sgtable_for_cpu(x->dma_dev, x->sgt, x->dir);
		dma_buf_unmap_attachment(x->attach, x->sgt, x->dir);
	}
	if (x->attach)
		dma_buf_detach(x->dmabuf, x->attach);
	if (x->dmabuf)
		dma_buf_put(x->dmabuf);
	return deliver ? 0 : (x->raw_ok ? -EIO : 0);
}

/* Wait for every posted cell to complete.  timeout_ms == 0 is a pure
 * poll: return -EAGAIN while any cell is pending, never sleep — the RCCL
 * plugin's control reader uses it to service RX completions between
 * control messages.  RX callbacks charge the owning transfer directly, so
 * several NOWAIT transfers can be in flight on the same rings without one
 * token's callbacks satisfying another token's wait. */
static int odl_tb5_dmabuf_wait(struct odl_tb5_device *dev,
			       struct odl_tb5_dmabuf_xfer *x, int timeout_ms)
{
	int p, ret = 0;

	for (p = 0; p < x->nps; p++) {
		struct odl_tb5_ring_ctx *rc =
			x->is_tx ? &dev->paths[p].tx : &dev->paths[p].rx;
		long need = x->is_tx ? x->base[p] + x->fidx[p]
				     : x->fidx[p];

		if (x->fidx[p] == 0)
			continue;
		if (timeout_ms == 0) {
			bool pending = x->is_tx ?
				atomic_read(&rc->completed) < need :
				atomic_read(&x->done[p]) < need;

			if (pending) {
				pr_debug_ratelimited("odl_tb5: dmabuf RX token pending path=%d own=%d/%d ring=%d/%d\n",
					p, atomic_read(&x->done[p]), x->fidx[p],
					atomic_read(&rc->completed),
					atomic_read(&rc->submitted));
				ret = -EAGAIN;
				break;
			}
			continue;
		}
		ret = wait_event_interruptible_timeout(rc->waitq,
			x->is_tx ? atomic_read(&rc->completed) >= need :
				   atomic_read(&x->done[p]) >= need,
			msecs_to_jiffies(timeout_ms));
		if (ret == 0) {
			/* Bounded wait.  A completion that never lands used to
			 * pin the ioctl indefinitely (the dmabuf transport
			 * hang).  Time out, reclaim every posted ring
			 * (stop+start, in finish) so the DMA mapping is never
			 * dropped under in-flight frames, and surface
			 * -ETIMEDOUT so the caller can measure the failure
			 * instead of hanging forever.  Report the completion-
			 * pump state, not just the counters: a stalled wait is
			 * either "the pump was never armed" (poll_active == 0)
			 * or "it is armed and still sees nothing", and the two
			 * have different fixes.  tx_inflight is the arming
			 * input -- odl_tb5_tx_submitted() only kicks the poll
			 * on the 0->1 edge, so a leaked count silently disables
			 * arming for every later submit. */
			pr_warn("odl_tb5: dmabuf %s wait timeout on path %d "
				"(completed=%d submitted=%d tx_inflight=%d "
				"poll_active=%d poll_idle_ticks=%u e2e=%d); "
				"ring_head=%d ring_tail=%d ring_running=%d; "
				"reclaiming frames via ring stop\n",
				x->is_tx ? "TX" : "RX", p,
				atomic_read(&rc->completed),
				atomic_read(&rc->submitted),
				atomic_read(&dev->tx_inflight),
				atomic_read(&dev->poll_active),
				dev->poll_idle_ticks,
				odl_e2e ? 1 : 0,
				READ_ONCE(rc->ring->head),
				READ_ONCE(rc->ring->tail),
				rc->ring->running ? 1 : 0);
			ret = -ETIMEDOUT;
			break;
		}
		if (ret == -ERESTARTSYS) {
			/* A signal arrived mid-DMA.  The release below must not
			 * race the still in-flight transfer (K3), so wait
			 * uninterruptibly for it to drain -- but bound the
			 * wait.  odl_tb5_dmabuf_reclaim drains and keeps the
			 * ring running; only a wedged ring gets stop+started. */
			if (!odl_tb5_dmabuf_reclaim(rc, x->base[p],
						    x->fidx[p],
						    x->is_tx ? NULL : &x->done[p],
						    false) &&
			    x->rx_shared)
				dev->paths[p].rx_target = 0;
			ret = (x->is_tx ?
			       atomic_read(&rc->completed) >=
				       x->base[p] + x->fidx[p] :
			       atomic_read(&x->done[p]) >= x->fidx[p]) ? 0 : -EIO;
		}
	}
	return ret;
}

/* Complete a parked transfer.  On error the in-flight frames are drained
 * (reclaim) before the mapping is dropped so the teardown cannot race
 * the DMA (teardown UAF (B)); on success the payload is validated and
 * copied back and the transfer returned clean. */
static int odl_tb5_dmabuf_finish(struct odl_tb5_device *dev,
				 struct odl_tb5_dmabuf_xfer *x, int ret)
{
	int p;

	if (ret != 0) {
		for (p = 0; p < x->nps; p++) {
			struct odl_tb5_ring_ctx *rc =
				x->is_tx ? &dev->paths[p].tx
					 : &dev->paths[p].rx;

			if (x->fidx[p] == 0 || !rc->ring || !rc->started)
				continue;
			if (!odl_tb5_dmabuf_reclaim(rc, x->base[p],
						    x->fidx[p],
						    x->is_tx ? NULL : &x->done[p],
						    false) &&
			    x->rx_shared)
				dev->paths[p].rx_target = 0;
		}
		odl_tb5_dmabuf_release(dev, x, false);
		return ret;
	}
	return odl_tb5_dmabuf_release(dev, x, true);
}

static int odl_tb5_dmabuf_submit(struct odl_tb5_device *dev,
				 int dmabuf_fd, loff_t offset, size_t len,
				 bool is_tx, struct odl_tb5_dmabuf_xfer *x)
{
	/* Keep the attachment bidirectional while validating the caller's mapped
	 * layout.  The actual wire frames are copied through coherent pool slots;
	 * explicit ownership synchronization keeps the CPU view correct. */
	enum dma_data_direction dir = DMA_BIDIRECTIONAL;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	struct scatterlist *sg;
	struct device *dma_dev;
	size_t sg_remaining, chunk;
	size_t total_remaining;
	loff_t skip;
	int fidx[ODL_TB5_MAX_PATHS] = { 0 };	/* per-path frame index */
	int needed[ODL_TB5_MAX_PATHS] = { 0 }; /* dry-run descriptors per path */
	long base[ODL_TB5_MAX_PATHS] = { 0 };	/* per-path completion baseline */
	int nps, ndp, p, k = 0;
	int nents_i;
	int ret = 0;
	bool mapped = false;	/* attach+map completed; guards release() */
	bool rx_shared = false;	/* single-path fallback: dmabuf shares ctrl path */
	bool rx_armed = false;	/* dmabuf_rx_active incremented for this call */
	bool rx_poll_held = false; /* rx_dmabuf_pending incremented for this call */
	int rx_pool_posted = 0;
	struct odl_tb5_dmabuf_stage *stage = NULL;
	struct iosys_map cpu_map = IOSYS_MAP_INIT_VADDR(NULL);
	bool cpu_access = false;
	bool cpu_mapped = false;
	int stage_count = 0;
	int total_needed = 0;
	/* Raw-payload zero-copy for this transfer.  Negotiated per link (S1);
	 * the geometry gate below can further drop it to framed.  Reads are
	 * unsynchronized with the handshake writers — the window between a
	 * logout reset and the next handshake can leave a stale true here,
	 * which the rx_len-mismatch validation catches; WRITE_ONCE/READ_ONCE
	 * keep the value defined. */
	bool raw = READ_ONCE(dev->raw_payload_ok);
	bool raw_ok = false;

	for (p = 0; p < ODL_TB5_MAX_PATHS; p++)
		atomic_set(&x->done[p], 0);

	if (dev->state != ODL_TB5_STATE_CONNECTED &&
	    dev->state != ODL_TB5_STATE_READY)
		return -ENOTCONN;

	/* Stripe across the negotiated (hopid-allocated, ring-started) paths.
	 * This is the symmetric count both peers agree on; clamp defensively.
	 * odl_dmabuf_paths (default 0) can cap it for A/B diagnostics. */
	nps = dev->negotiated_paths;
	if (nps < 1)
		nps = 1;
	if (nps > dev->num_paths)
		nps = dev->num_paths;
	if (odl_dmabuf_paths && (int)odl_dmabuf_paths < nps)
		nps = odl_dmabuf_paths;

	/* dmabuf ring separation: reserve path 0 as the control/pool/stream-
	 * header path and stripe dmabuf payload only across dmabuf-only paths
	 * 1..(nps-1), so pool/header frames and dmabuf frames never share a
	 * ring FIFO.  Path 0 carries the transfer's own stream header, so it
	 * MUST NOT also carry dmabuf payload (that collides and stalls the
	 * ring).  Both peers compute the same nps (symmetric negotiated value)
	 * and E2E pairs path-p TX with path-p RX, so they agree by
	 * construction.  With a single negotiated path there is no spare ring:
	 * dmabuf shares the control path and a dmabuf_rx_active guard in
	 * odl_tb5_rx_repost suppresses pool auto-repost during the transfer.
	 * To get multi-path dmabuf bandwidth, run with num_paths>=3 so paths
	 * 1..(nps-1) give two or more dmabuf-only rings. */
	ndp = (nps >= 2) ? (nps - 1) : 0;
	rx_shared = (!is_tx && ndp == 0);

/* Staging frames carry a fixed-size internal header, so an absolute
	 * payload ceiling is known before attaching the caller's dma-buf.  Reject
	 * an impossible transfer here: mapping a grossly oversized buffer can
	 * invalidate live NHI translations even though no ring descriptor is
	 * eventually posted.  The SG-layout preflight below still handles
	 * smaller buffers whose segment boundaries need extra descriptors.
	 * FRAME_LEN_MAX is the largest payload a single ring slot has ever
	 * carried (raw cells are RAW_CELL_MAX, framed payload cells
	 * STREAM_PAYLOAD_MAX, both smaller), so it is a valid per-slot bound
	 * regardless of the negotiated raw mode. */
	{
		u64 capacity = 0;
		size_t frame_max = ODL_TB5_FRAME_LEN_MAX;
		int first = ndp ? 1 : 0;
		int last = ndp ? ndp : 0;

		for (p = first; p <= last; p++) {
			struct odl_tb5_ring_ctx *rc =
				is_tx ? &dev->paths[p].tx : &dev->paths[p].rx;

			capacity += (u64)rc->ring_size * frame_max;
		}
		if ((u64)len > capacity)
			return -ENOSPC;
	}

	dmabuf = dma_buf_get(dmabuf_fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	if (offset < 0 || len == 0 ||
	    (size_t)offset > dmabuf->size ||
	    len > dmabuf->size - (size_t)offset) {
		ret = -EINVAL;
		goto err_put;
	}

	/* Attach to the NHI DMA device (what actually programs the rings),
	 * NOT dev->dev (the character device, which has no DMA ops / IOMMU
	 * domain — mapping against it yields no usable DMA segments and the
	 * transfer silently posts zero frames).  All paths share one NHI, so a
	 * single attach/map produces DMA addresses usable by every path ring. */
	dma_dev = tb_ring_dma_device(is_tx ? dev->paths[0].tx.ring
					   : dev->paths[0].rx.ring);
	attach = dma_buf_attach(dmabuf, dma_dev);
	if (IS_ERR(attach)) {
		pr_warn_ratelimited("odl_tb5: dma_buf_attach(fd=%d dev=%s) failed: %ld\n",
				    dmabuf_fd, dev_name(dma_dev),
				    PTR_ERR(attach));
		ret = PTR_ERR(attach);
		goto err_put;
	}

	sgt = dma_buf_map_attachment(attach, dir);
	if (IS_ERR(sgt)) {
		pr_warn_ratelimited("odl_tb5: dma_buf_map_attachment(size=%zu dev=%s) failed: %ld\n",
				    (size_t)dmabuf->size, dev_name(dma_dev),
				    PTR_ERR(sgt));
		ret = PTR_ERR(sgt);
		goto err_detach;
	}
	dma_sync_sgtable_for_device(dma_dev, sgt, dir);
	mapped = true;

	/* Hold the completion pump before the first RX frame can be posted.
	 * This also covers partial-post failures: err_unmap may need to wait for
	 * already queued frames before it can safely release this mapping. */
	if (!is_tx) {
		atomic_inc(&dev->rx_dmabuf_pending);
		rx_poll_held = true;
		odl_tb5_poll_kick(dev);
	}

	/* RX ring separation: in multi-path mode the dmabuf paths (1..ndp)
	 * carry no pool/header frames (odl_tb5_rx_arm arms only the control
	 * path 0, and stream headers travel on path 0), so these rings are
	 * exclusively ours.  In the single-path fallback dmabuf shares path 0
	 * with the pool; we then raise dmabuf_rx_active so odl_tb5_rx_repost
	 * suppresses pool auto-repost during the transfer, keeping the ring
	 * empty for the raw payload.  (An earlier attempt to tb_ring_stop/
	 * start-flush the pool frames mid-op broke RX reception outright —
	 * never disturb a live RX ring mid-op; the dmabuf_rx_active guard
	 * avoids that.)
	 *
	 * The guard alone only stops *new* pool frames from queueing; frames
	 * already posted into the path-0 RX FIFO (the pool window armed by
	 * odl_tb5_rx_arm, up to half the ring depth) still sit ahead of the
	 * payload.  They are counted into rx_pool_posted below so the
	 * capacity preflight cannot over-commit the ring. */
	if (rx_shared) {
		struct odl_tb5_ring_ctx *rx_rc = &dev->paths[0].rx;
		unsigned long flags;

		/* Serialize the ownership handoff with RX pool repost.  Once the
		 * flag is visible, callbacks may drain existing pool frames but no
		 * new pool frame can enter this ring until cleanup drops it. */
		spin_lock_irqsave(&rx_rc->lock, flags);
		atomic_inc(&dev->dmabuf_rx_active);
		rx_armed = true;
		spin_unlock_irqrestore(&rx_rc->lock, flags);
		/* A refill admitted before the guard was raised may still be
		 * posting.  Let it finish before taking the occupancy snapshot. */
		wait_event(dev->paths[0].rx_repost_waitq,
			   atomic_read(&dev->paths[0].rx_reposting) == 0);

		/* Drop the pool window for the duration of the single-path
		 * session so neither this transfer nor the next posts
		 * payload behind a pool window.  odl_tb5_rx_arm re-arms
		 * when stream traffic actually needs the pool again.
		 *
		 * Note we must NOT try to force-empty the window with
		 * tb_ring_stop/start — that is the exact "poison" the
		 * timeout path describes: a re-started ring resumes an NHI
		 * descriptor position that no longer matches the driver's,
		 * so the next submit stalls or stale descriptors "complete"
		 * spuriously (measured interleaved 5/10 corruption when
		 * attempted).  Frames already posted are left to be consumed
		 * by inbound traffic and are accounted for by the
		 * rx_pool_posted snapshot below. */
		dev->paths[0].rx_target = 0;

		/* Payload-only rings have no pool occupancy.  Path 0 is different:
		 * pool frames already posted before the
		 * handoff still occupy hardware slots.  Account for that live
		 * occupancy before posting any payload, avoiding a partial submit
		 * followed by -ENOSPC.  The repost admission protocol plus
		 * dmabuf_rx_active excludes new pool posts, while callbacks can only
		 * reduce this count, so the snapshot is conservative. */
		rx_pool_posted = atomic_read(&dev->paths[0].rx_posted);
		if (WARN_ON_ONCE(rx_pool_posted < 0))
			rx_pool_posted = 0;
		if (rx_pool_posted > rx_rc->ring_size)
			rx_pool_posted = rx_rc->ring_size;
	}

	/* Capacity preflight over the mapped layout.  DMA segments can end
	 * mid-cell, so DIV_ROUND_UP(len, cell-max) undercounts when a
	 * transfer crosses scatterlist boundaries.  Mirror the real chunking and
	 * block-stripe choice, then compare each selected direction/ring before
	 * posting even one descriptor.  In the shared RX fallback the admission
	 * handshake above makes rx_pool_posted stable against refills; callbacks
	 * may only drain it, making this snapshot conservative. */
	raw_ok = raw;
	if (raw_ok && ((size_t)offset % PAGE_SIZE) != 0) {
		/* A mid-page start cannot be posted as a descriptor cell; the
		 * peer would absorb it at a different offset. */
		raw_ok = false;
		ODL_STAT_INC(dev, raw_unaligned_fallback);
	}
	{
		/* First pass: candidate raw chunking (RAW_CELL_MAX cells).  The
		 * gate verdict and the descriptor counts must come from the SAME
		 * geometry that the submit loop will actually use, so when the
		 * gate fails we re-walk with the framed cap instead of
		 * patching counts after the fact. */
		if (raw && raw_ok) {
			ret = odl_tb5_dmabuf_walk(sgt, offset, len, ndp,
						  is_tx, dev, rx_pool_posted,
						  rx_shared,
						  ODL_TB5_RAW_CELL_MAX,
						  needed, &raw_ok);
			if (ret == -ENOSPC) {
				/* Raw cells are half the size of framed payload
				 * cells, so a ring that cannot hold the raw
				 * descriptor count may still fit the same
				 * transfer framed.  Fall back rather than
				 * failing a transfer that would otherwise go
				 * through (the shared `if (!raw_ok)` path below
				 * re-walks with the framed cap). */
				raw_ok = false;
			} else if (ret) {
				goto err_unmap;
			}
			if (!raw_ok) {
				ODL_STAT_INC(dev, raw_eligible_reject);
				memset(needed, 0, sizeof(needed));
				ret = odl_tb5_dmabuf_walk(sgt, offset, len,
							  ndp, is_tx, dev,
							  rx_pool_posted,
							  rx_shared,
							  ODL_TB5_STREAM_PAYLOAD_MAX,
							  needed, NULL);
				if (ret)
					goto err_unmap;
			}
		} else {
			ret = odl_tb5_dmabuf_walk(sgt, offset, len, ndp,
						  is_tx, dev, rx_pool_posted,
						  rx_shared,
						  ODL_TB5_STREAM_PAYLOAD_MAX,
						  needed, NULL);
			if (ret)
				goto err_unmap;
		}
		/* The logout→re-handshake window can flip the negotiated flag
		 * between the read above and the preflight walk.  Raw TX
		 * frames carry no in-band header: a peer that restarted to a
		 * framed-only session would mis-parse the exporter bytes as
		 * stream headers (silent wrong-data delivery — RX has the
		 * length-sum validation, TX has nothing).  Re-check after
		 * the walk and re-walk framed when the flag fell. */
		if (raw_ok && !READ_ONCE(dev->raw_payload_ok)) {
			raw_ok = false;
			ODL_STAT_INC(dev, raw_eligible_reject);
			memset(needed, 0, sizeof(needed));
			ret = odl_tb5_dmabuf_walk(sgt, offset, len, ndp,
						  is_tx, dev, rx_pool_posted,
						  rx_shared,
						  ODL_TB5_STREAM_PAYLOAD_MAX,
						  needed, NULL);
			if (ret)
				goto err_unmap;
		}
	}
	for (p = 0; p < nps; p++)
		total_needed += needed[p];
	stage = kcalloc(total_needed, sizeof(*stage), GFP_KERNEL);
	if (!stage) {
		ret = -ENOMEM;
		goto err_unmap;
	}
	/* Raw cells are DMAed straight into the exporter pages — no CPU
	 * mapping is needed (and dma_buf_vmap on some exporters forces a
	 * bounce).  The framed path still stages through the pool. */
	if (!raw_ok) {
		ret = dma_buf_begin_cpu_access(dmabuf, DMA_BIDIRECTIONAL);
		if (ret)
			goto err_unmap;
		cpu_access = true;
		ret = dma_buf_vmap(dmabuf, &cpu_map);
		if (ret)
			goto err_unmap;
		cpu_mapped = true;
	}

	skip = offset;
	total_remaining = len;

	for_each_sgtable_dma_sg(sgt, sg, nents_i) {
		sg_remaining = sg_dma_len(sg);

		if (skip > 0) {
			if ((size_t)skip >= sg_remaining) {
				skip -= sg_remaining;
				continue;
			}
			sg_remaining -= skip;
			skip = 0;
		}

		while (sg_remaining > 0 && total_remaining > 0) {
			struct odl_tb5_ring_ctx *rc;
			struct ring_frame *frame;
			struct odl_tb5_dmabuf_stage *bounce;
			size_t sent = len - total_remaining;
			int occupied;

			/* dmabuf ring separation: in multi-path mode stripe
			 * across the dmabuf-only paths 1..ndp so these frames
			 * never land on the control/pool/header path 0.  Block
			 * striping keeps each path's writes to one contiguous
			 * region (two DMA engines hammering adjacent
			 * cache-line-sharing memory corrupts frame-boundary bytes
			 * under load); both peers compute the same `sent` → same
			 * dmabuf path, and E2E pairs path-p TX with path-p RX so it
			 * lands at the matching offset.  In the single-path
			 * fallback (ndp==0) dmabuf shares the control path under
			 * the dmabuf_rx_active repost guard. */
			p = odl_tb5_dmabuf_path(ndp, len, sent);
			rc = is_tx ? &dev->paths[p].tx : &dev->paths[p].rx;
			occupied = (!is_tx && rx_shared && p == 0) ?
				   rx_pool_posted : 0;
			if (occupied >= rc->ring_size) {
				/* Should be unreachable: the preflight fully
				 * accounts for the rx_pool occupancy. */
				ret = -EIO;
				goto err_unmap;
			}

			if (!fidx[p])
				/* Submits are serialized by the direction lock, so
				 * submitted is this transfer's stable position in the
				 * completion FIFO.  Using completed gives concurrent
				 * NOWAIT transfers overlapping wait ranges. */
				base[p] = atomic_read(&rc->submitted);

			if (fidx[p] >= rc->ring_size) {
				ret = -ENOSPC;
				goto err_unmap;
			}

chunk = min3(raw_ok ? (size_t)ODL_TB5_RAW_CELL_MAX
				    : (size_t)ODL_TB5_STREAM_PAYLOAD_MAX,
			     sg_remaining, total_remaining);
			bounce = &stage[stage_count];
			if (stage_count >= total_needed) {
				/* The submit loop must never outrun the walk's
				 * descriptor count — same chunking by
				 * construction; fail loudly on drift. */
				ret = -EIO;
				goto err_unmap;
			}
			bounce->offset = (size_t)offset + sent;
			bounce->len = chunk;
			bounce->xfer = x;
			bounce->path = p;
			if (raw_ok) {
				bounce->seg_off = sg_dma_len(sg) - sg_remaining;
				bounce->dma = sg_dma_address(sg) +
					      bounce->seg_off;
				/* Zero-copy: the NHI DMAes the exporter pages
				 * directly — no pool slot, no in-band header,
				 * no memcpy.  The frame identity is the posted
				 * descriptor.  The EOF marker stays the
				 * hardware DATA value: an EOF_RAW marker on
				 * the wire made the NHI drop the E2E
				 * credit-return for the first posted frame
				 * (tx_inflight stuck at 1 for the ring
				 * lifetime); raw-ness is negotiated, not
				 * wire-marked. */
				memset(&bounce->frame, 0,
				       sizeof(bounce->frame));
				bounce->frame.buffer_phy = bounce->dma;
				bounce->frame.sof = ODL_TB5_PDF_SOF_DATA;
				bounce->frame.eof = ODL_TB5_PDF_EOF_DATA;
				if (is_tx) {
					bounce->frame.size = chunk;
					bounce->frame.callback =
						odl_tb5_tx_dmabuf_callback;
				} else {
					/* Advertise the complete 4096-byte cell
					 * (encoded as zero in the 12-bit NHI
					 * descriptor).  The peer transmits at
					 * most RAW_CELL_MAX (2048), well inside
					 * the advertised slot, so the descriptor
					 * cannot be overrun. */
					bounce->frame.size = 0;
					bounce->frame.callback =
						odl_tb5_rx_dmabuf_raw_callback;
				}
				frame = &bounce->frame;
			} else {
				/* Leave room for the private frame header
				 * inside the NHI's 4096-byte descriptor
				 * limit. */
				bounce->slot = odl_tb5_frame_pool_get(
						&dev->frame_pool);
				if (!bounce->slot) {
					ret = -ENOMEM;
					goto err_unmap;
				}
				bounce->cpu = bounce->slot->virt;
				bounce->dma = bounce->slot->phys;
				if (!is_tx) {
					bounce->slot->dmabuf_xfer = x;
					bounce->slot->dmabuf_path = p;
				}
				if (is_tx) {
					struct odl_tb5_stream_hdr *hdr =
						bounce->cpu;

					memset(hdr, 0, sizeof(*hdr));
					hdr->flags = ODL_TB5_SHDR_F_SINGLE;
					hdr->frag_idx = cpu_to_le16(stage_count);
					hdr->payload_len = cpu_to_le16(chunk);
					iosys_map_memcpy_from(
						bounce->cpu +
							ODL_TB5_STREAM_HDR_SIZE,
						&cpu_map, bounce->offset,
						chunk);
				}
				frame = &bounce->slot->frame;
				frame->buffer_phy = bounce->dma;
				/* TX advertises the payload length.  RX must
				 * advertise the complete 4096-byte slot
				 * (encoded as zero in the 12-bit NHI
				 * descriptor), because the link's framing is
				 * counted against receive capacity.
				 * Advertising only `chunk` makes a maximum
				 * payload overrun the descriptor and the NHI
				 * drops it without a callback.  The peer still
				 * transmits only `chunk` bytes, so the mapped
				 * payload range is all that is written. */
				frame->size = is_tx ?
					ODL_TB5_STREAM_HDR_SIZE + chunk : 0;
				frame->callback = is_tx ?
					odl_tb5_tx_dmabuf_callback
					: odl_tb5_rx_dmabuf_callback;
				frame->sof = ODL_TB5_PDF_SOF_DATA;
				frame->eof = ODL_TB5_PDF_EOF_DATA;
			}
			stage_count++;

			ret = is_tx ? tb_ring_tx(rc->ring, frame)
				    : tb_ring_rx(rc->ring, frame);
			if (ret < 0)
				goto err_unmap;
			if (is_tx)
				odl_tb5_tx_submitted(dev);
			if (is_tx) {
				atomic64_inc(&dev->stats.path_tx_frames[p]);
				if (raw_ok)
					ODL_STAT_INC(dev, raw_tx_frames);
			} else {
				atomic64_inc(&dev->stats.path_rx_frames[p]);
			}

			sg_remaining -= chunk;
			total_remaining -= chunk;
			fidx[p]++;
			k++;
		}

		if (total_remaining == 0)
			break;
	}

	/* No frames posted means the dmabuf produced no usable DMA segments
	 * (e.g. mapped against the wrong device).  Fail loudly rather than
	 * satisfying the completion wait trivially and returning a silent
	 * no-op — that masked a broken transport for the whole dmabuf path. */
	if (k == 0) {
		ret = -EIO;
		goto err_unmap;
	}

	/* Publish per-path submitted counts (deferred until here so an early
	 * tb_ring failure leaves them untouched, as the single-path code did). */
	for (p = 0; p < nps; p++) {
		struct odl_tb5_ring_ctx *rc =
			is_tx ? &dev->paths[p].tx : &dev->paths[p].rx;

		if (fidx[p] > 0)
			atomic_add(fidx[p], &rc->submitted);
	}

	/* Hand the transfer to the caller: wait() and finish() run later,
	 * possibly from a different syscall (NOWAIT submit).  Park every
	 * resource and counter so they never touch this stack frame again. */
	x->is_tx = is_tx;
	x->mapped = mapped;
	x->nps = nps;
	x->ndp = ndp;
	x->rx_shared = rx_shared;
	x->rx_armed = rx_armed;
	x->rx_poll_held = rx_poll_held;
	x->rx_pool_posted = rx_pool_posted;
	x->dmabuf = dmabuf;
	x->attach = attach;
	x->sgt = sgt;
	x->dma_dev = dma_dev;
	x->dir = dir;
	x->stage = stage;
	x->stage_count = stage_count;
	x->cpu_map = cpu_map;
	x->cpu_access = cpu_access;
	x->cpu_mapped = cpu_mapped;
	x->raw_ok = raw_ok;
	x->len = len;
	x->posted_total = k;
	memcpy(x->base, base, sizeof(base));
	memcpy(x->fidx, fidx, sizeof(fidx));
	return 0;

err_unmap:
	/* teardown UAF (B): any frames we already posted are still queued in
	 * the thunderbolt ring; they must be out of in-flight before we drop
	 * the dmabuf mapping.  Drain them cleanly (wait for our completions)
	 * and keep the ring running when possible.  tb_ring_stop/start
	 * resets the driver-side head to 0 but leaves the NHI controller's
	 * internal descriptor position untouched, so a re-started ring makes
	 * the DMA engine resume from a stale slot: new posts behind the
	 * engine's head never complete and stale descriptors in its path
	 * "complete" without transmitting (peer sees rx_frames_seen == 0).
	 * The 16 MiB ring-full reject takes this path after every in-process
	 * transfer, so a stop+start here poisons the very next submit (F2).
	 * Only escalate to stop+start when the ring is genuinely wedged
	 * (drain timeout) and its frames must be forcibly canceled. */
	for (p = 0; p < nps; p++) {
		struct odl_tb5_ring_ctx *rc =
			is_tx ? &dev->paths[p].tx : &dev->paths[p].rx;

		if (fidx[p] == 0 || !rc->ring || !rc->started)
			continue;

		/* RX teardown follows this call's private callbacks; TX teardown
		 * follows its serialized ring FIFO range. */
		if (!odl_tb5_dmabuf_reclaim(rc, base[p], fidx[p],
					      is_tx ? NULL : &x->done[p], true) &&
		    rx_shared)
			dev->paths[p].rx_target = 0;
	}
	x->is_tx = is_tx;
	x->mapped = mapped;
	x->nps = nps;
	x->ndp = ndp;
	x->rx_shared = rx_shared;
	x->rx_armed = rx_armed;
	x->rx_poll_held = rx_poll_held;
	x->rx_pool_posted = rx_pool_posted;
	x->dmabuf = dmabuf;
	x->attach = attach;
	x->sgt = sgt;
	x->dma_dev = dma_dev;
	x->dir = dir;
	x->stage = stage;
	x->stage_count = stage_count;
	x->cpu_map = cpu_map;
	x->cpu_access = cpu_access;
	x->cpu_mapped = cpu_mapped;
	x->raw_ok = raw_ok;
	x->len = len;
	x->posted_total = k;
	memcpy(x->base, base, sizeof(base));
	memcpy(x->fidx, fidx, sizeof(fidx));
	odl_tb5_dmabuf_release(dev, x, false);
	return ret;

err_detach:
	dma_buf_detach(dmabuf, attach);
err_put:
	dma_buf_put(dmabuf);
	return ret;
}

int odl_tb5_submit_tx_dmabuf(struct odl_tb5_device *dev,
			     int dmabuf_fd, loff_t offset, size_t len)
{
	struct odl_tb5_dmabuf_xfer x;
	int ret;

	/* Serialize TX callers independently from RX so send and receive can
	 * still run concurrently.  The whole transfer stays inside the lock
	 * (blocking wait), matching the pre-split behavior. */
	mutex_lock(&dev->dmabuf_tx_lock);
	memset(&x, 0, sizeof(x));
	ret = odl_tb5_dmabuf_submit(dev, dmabuf_fd, offset, len, true, &x);
	if (ret == 0)
		ret = odl_tb5_dmabuf_wait(dev, &x, 5000);
	ret = odl_tb5_dmabuf_finish(dev, &x, ret);
	mutex_unlock(&dev->dmabuf_tx_lock);
	return ret;
}

int odl_tb5_submit_rx_dmabuf(struct odl_tb5_device *dev,
			     int dmabuf_fd, loff_t offset, size_t len)
{
	struct odl_tb5_dmabuf_xfer x;
	int ret;

	/* The synchronous path's capacity snapshot assumes no second dmabuf
	 * receive is posting to the same selected rings. */
	mutex_lock(&dev->dmabuf_rx_lock);
	memset(&x, 0, sizeof(x));
	ret = odl_tb5_dmabuf_submit(dev, dmabuf_fd, offset, len, false, &x);
	if (ret == 0)
		ret = odl_tb5_dmabuf_wait(dev, &x, 5000);
	ret = odl_tb5_dmabuf_finish(dev, &x, ret);
	mutex_unlock(&dev->dmabuf_rx_lock);
	return ret;
}

/* NOWAIT receive: post the RX cells and return a token; the caller polls
 * completion with odl_tb5_wait_rx_dmabuf().  Never blocks in the transfer
 * wait — required by the RCCL plugin's control reader, which would
 * otherwise stop servicing the peer's control messages (structural
 * deadlock).  Multiple NOWAIT transfers can be in flight; each waits on
 * its own per-path baseline. */
int odl_tb5_submit_rx_dmabuf_nowait(struct odl_tb5_device *dev,
				    int dmabuf_fd, loff_t offset, size_t len,
				    int *token)
{
	struct odl_tb5_dmabuf_xfer *x;
	int idx, ret;

	if (!token)
		return -EINVAL;

	mutex_lock(&dev->dmabuf_rx_lock);
	for (idx = 0; idx < ODL_TB5_DMABUF_MAX_PENDING; idx++)
		if (dev->dmabuf_rx_pending[idx].state == ODL_DMABUF_SLOT_FREE)
			break;
	if (idx == ODL_TB5_DMABUF_MAX_PENDING) {
		mutex_unlock(&dev->dmabuf_rx_lock);
		return -EBUSY;
	}
	x = &dev->dmabuf_rx_pending[idx];
	memset(x, 0, sizeof(*x));
	x->state = ODL_DMABUF_SLOT_BUSY;
	ret = odl_tb5_dmabuf_submit(dev, dmabuf_fd, offset, len, false, x);
	if (ret) {
		x->state = ODL_DMABUF_SLOT_FREE;
		mutex_unlock(&dev->dmabuf_rx_lock);
		return ret;
	}
	*token = idx + 1;
	mutex_unlock(&dev->dmabuf_rx_lock);
	return 0;
}

/* Wait for a NOWAIT transfer.  timeout_ms == 0: poll once; -EAGAIN while
 * pending (the slot stays armed for another poll).  On timeout the slot
 * is reclaimed (drain + ring stop/start if wedged) and freed, matching
 * the blocking path's error behavior. */
int odl_tb5_wait_rx_dmabuf(struct odl_tb5_device *dev, int token,
			   int timeout_ms)
{
	struct odl_tb5_dmabuf_xfer *x;
	int idx, ret;

	if (token <= 0 || token > ODL_TB5_DMABUF_MAX_PENDING)
		return -ENOENT;
	idx = token - 1;

	mutex_lock(&dev->dmabuf_rx_lock);
	x = &dev->dmabuf_rx_pending[idx];
	if (x->state == ODL_DMABUF_SLOT_FREE) {
		mutex_unlock(&dev->dmabuf_rx_lock);
		return -ENOENT;
	}
	if (x->state != ODL_DMABUF_SLOT_BUSY) {
		/* A WAIT already owns the slot; only one waiter may poll it. */
		mutex_unlock(&dev->dmabuf_rx_lock);
		return -EBUSY;
	}
	x->state = ODL_DMABUF_SLOT_WAITING;
	mutex_unlock(&dev->dmabuf_rx_lock);

	ret = odl_tb5_dmabuf_wait(dev, x, timeout_ms);
	if (ret == -EAGAIN) {
		mutex_lock(&dev->dmabuf_rx_lock);
		x->state = ODL_DMABUF_SLOT_BUSY;
		mutex_unlock(&dev->dmabuf_rx_lock);
		return -EAGAIN;
	}

	mutex_lock(&dev->dmabuf_rx_lock);
	ret = odl_tb5_dmabuf_finish(dev, x, ret);
	x->state = ODL_DMABUF_SLOT_FREE;
	mutex_unlock(&dev->dmabuf_rx_lock);
	return ret;
}

/* ══════════════════════════════════════════════════════════════════════
 * DMA Frame Pool
 * ══════════════════════════════════════════════════════════════════════ */

int odl_tb5_frame_pool_alloc(struct odl_tb5_device *dev, int size)
{
	struct odl_tb5_frame_pool *pool = &dev->frame_pool;
	struct device *dma_dev = tb_ring_dma_device(dev->paths[0].tx.ring);
	int i;

	pool->size = size;
	pool->free_count = pool->size;
	spin_lock_init(&pool->lock);
	init_waitqueue_head(&pool->avail_waitq);

	pool->slots = kvzalloc(pool->size * sizeof(*pool->slots), GFP_KERNEL);
	if (!pool->slots)
		return -ENOMEM;

	pool->bitmap = bitmap_zalloc(pool->size, GFP_KERNEL);
	if (!pool->bitmap) {
		kvfree(pool->slots);
		pool->slots = NULL;
		return -ENOMEM;
	}

	for (i = 0; i < pool->size; i++) {
		struct odl_tb5_frame_slot *slot = &pool->slots[i];

		slot->virt = dma_alloc_coherent(dma_dev, ODL_TB5_FRAME_SIZE,
						&slot->phys, GFP_KERNEL);
		if (!slot->virt) {
			pr_err("odl_tb5: frame pool alloc failed at slot %d\n", i);
			goto err_free;
		}
		slot->slot_idx = i;
		slot->in_use = false;
		slot->frame.buffer_phy = slot->phys;
	}

	pr_info("odl_tb5: frame pool allocated: %d x %d bytes (%d KB)\n",
		pool->size, ODL_TB5_FRAME_SIZE,
		(pool->size * ODL_TB5_FRAME_SIZE) >> 10);
	return 0;

err_free:
	while (--i >= 0) {
		dma_free_coherent(dma_dev, ODL_TB5_FRAME_SIZE,
				  pool->slots[i].virt, pool->slots[i].phys);
	}
	bitmap_free(pool->bitmap);
	pool->bitmap = NULL;
	kvfree(pool->slots);
	pool->slots = NULL;
	return -ENOMEM;
}

void odl_tb5_frame_pool_free(struct odl_tb5_device *dev)
{
	struct odl_tb5_frame_pool *pool = &dev->frame_pool;
	struct device *dma_dev;
	int i;

	if (!pool->slots)
		return;

	dma_dev = tb_ring_dma_device(dev->paths[0].tx.ring);

	for (i = 0; i < pool->size; i++) {
		if (pool->slots[i].virt)
			dma_free_coherent(dma_dev, ODL_TB5_FRAME_SIZE,
					  pool->slots[i].virt,
					  pool->slots[i].phys);
	}

	bitmap_free(pool->bitmap);
	pool->bitmap = NULL;
	kvfree(pool->slots);
	pool->slots = NULL;
}

struct odl_tb5_frame_slot *odl_tb5_frame_pool_get(struct odl_tb5_frame_pool *pool)
{
	struct odl_tb5_frame_slot *slot;
	unsigned long flags;
	int idx;

	spin_lock_irqsave(&pool->lock, flags);

	idx = find_first_zero_bit(pool->bitmap, pool->size);
	if (idx >= pool->size) {
		spin_unlock_irqrestore(&pool->lock, flags);
		return NULL;
	}

	set_bit(idx, pool->bitmap);
	slot = &pool->slots[idx];
	slot->in_use = true;
	slot->tx_msg = NULL;
	slot->dmabuf_xfer = NULL;
	slot->dmabuf_path = 0;
	pool->free_count--;

	spin_unlock_irqrestore(&pool->lock, flags);
	return slot;
}

void odl_tb5_frame_pool_put(struct odl_tb5_frame_pool *pool,
			    struct odl_tb5_frame_slot *slot)
{
	unsigned long flags;

	spin_lock_irqsave(&pool->lock, flags);

	clear_bit(slot->slot_idx, pool->bitmap);
	slot->in_use = false;
	slot->tx_msg = NULL;
	slot->dmabuf_xfer = NULL;
	slot->dmabuf_path = 0;
	pool->free_count++;

	spin_unlock_irqrestore(&pool->lock, flags);
	wake_up_interruptible(&pool->avail_waitq);
}

/* Batch allocation: get up to @requested slots with a single spinlock. */
int odl_tb5_frame_pool_get_batch(struct odl_tb5_frame_pool *pool,
				 struct odl_tb5_frame_slot **slots,
				 int requested)
{
	unsigned long flags;
	int allocated = 0;
	int idx = 0;

	spin_lock_irqsave(&pool->lock, flags);

	while (allocated < requested &&
	       pool->free_count > ODL_TB5_TX_POOL_RESERVE) {
		idx = find_next_zero_bit(pool->bitmap, pool->size, idx);
		if (idx >= pool->size)
			break;

		set_bit(idx, pool->bitmap);
		pool->free_count--;
		pool->slots[idx].in_use = true;
		pool->slots[idx].tx_msg = NULL;
		slots[allocated++] = &pool->slots[idx];
		idx++;
	}

	spin_unlock_irqrestore(&pool->lock, flags);
	return allocated;
}

/* ══════════════════════════════════════════════════════════════════════
 * SG Batch Buffer Pool (throughput mode)
 * ══════════════════════════════════════════════════════════════════════ */

int odl_tb5_batch_pool_alloc(struct odl_tb5_device *dev)
{
	struct odl_tb5_batch_pool *pool = &dev->batch_pool;
	struct device *dma_dev = tb_ring_dma_device(dev->paths[0].tx.ring);
	int i;

	INIT_LIST_HEAD(&pool->free_list);
	spin_lock_init(&pool->lock);
	init_waitqueue_head(&pool->avail_waitq);
	pool->free_count = 0;

	for (i = 0; i < ODL_TB5_BATCH_BUF_COUNT; i++) {
		struct odl_tb5_batch_buf *buf = &pool->bufs[i];

		buf->virt = dma_alloc_coherent(dma_dev,
					       ODL_TB5_BATCH_BUF_SIZE,
					       &buf->phys, GFP_KERNEL);
		if (!buf->virt) {
			pr_err("odl_tb5: batch buf alloc failed at %d\n", i);
			goto err_free;
		}

		buf->in_use = false;
		buf->tx_msg = NULL;
		atomic_set(&buf->frames_pending, 0);
		INIT_LIST_HEAD(&buf->list);
		list_add_tail(&buf->list, &pool->free_list);
		pool->free_count++;
	}

	pr_info("odl_tb5: batch pool allocated: %d x %d KB (%d MB total)\n",
		ODL_TB5_BATCH_BUF_COUNT,
		ODL_TB5_BATCH_BUF_SIZE >> 10,
		(ODL_TB5_BATCH_BUF_COUNT * ODL_TB5_BATCH_BUF_SIZE) >> 20);
	return 0;

err_free:
	while (--i >= 0) {
		dma_free_coherent(dma_dev, ODL_TB5_BATCH_BUF_SIZE,
				  pool->bufs[i].virt, pool->bufs[i].phys);
		pool->bufs[i].virt = NULL;
	}
	INIT_LIST_HEAD(&pool->free_list);
	pool->free_count = 0;
	return -ENOMEM;
}

void odl_tb5_batch_pool_free(struct odl_tb5_device *dev)
{
	struct odl_tb5_batch_pool *pool = &dev->batch_pool;
	struct device *dma_dev;
	int i;

	if (!pool->bufs[0].virt)
		return;

	if (dev->paths[0].tx.ring)
		dma_dev = tb_ring_dma_device(dev->paths[0].tx.ring);
	else if (dev->paths[0].rx.ring)
		dma_dev = tb_ring_dma_device(dev->paths[0].rx.ring);
	else
		return;

	for (i = 0; i < ODL_TB5_BATCH_BUF_COUNT; i++) {
		if (pool->bufs[i].virt)
			dma_free_coherent(dma_dev, ODL_TB5_BATCH_BUF_SIZE,
					  pool->bufs[i].virt,
					  pool->bufs[i].phys);
		pool->bufs[i].virt = NULL;
	}

	INIT_LIST_HEAD(&pool->free_list);
	pool->free_count = 0;
}

struct odl_tb5_batch_buf *
odl_tb5_batch_pool_get(struct odl_tb5_batch_pool *pool)
{
	struct odl_tb5_batch_buf *buf;
	unsigned long flags;

	spin_lock_irqsave(&pool->lock, flags);

	if (list_empty(&pool->free_list)) {
		spin_unlock_irqrestore(&pool->lock, flags);
		return NULL;
	}

	buf = list_first_entry(&pool->free_list,
			       struct odl_tb5_batch_buf, list);
	list_del_init(&buf->list);
	buf->in_use = true;
	pool->free_count--;

	spin_unlock_irqrestore(&pool->lock, flags);
	return buf;
}

void odl_tb5_batch_pool_put(struct odl_tb5_batch_pool *pool,
			    struct odl_tb5_batch_buf *buf)
{
	unsigned long flags;

	spin_lock_irqsave(&pool->lock, flags);

	buf->in_use = false;
	buf->tx_msg = NULL;
	list_add_tail(&buf->list, &pool->free_list);
	pool->free_count++;

	spin_unlock_irqrestore(&pool->lock, flags);
	wake_up_interruptible(&pool->avail_waitq);
}

/* ══════════════════════════════════════════════════════════════════════
 * Stream Lifecycle
 * ══════════════════════════════════════════════════════════════════════ */

static void odl_tb5_stream_free(struct kref *ref)
{
	struct odl_tb5_stream *stream =
		container_of(ref, struct odl_tb5_stream, refcount);
	struct odl_tb5_tx_msg *tx, *tx_tmp;
	struct odl_tb5_rx_msg *rx, *rx_tmp;

	list_for_each_entry_safe(tx, tx_tmp, &stream->tx_queue, list) {
		list_del(&tx->list);
		kvfree(tx->data);
		kfree(tx);
	}

	list_for_each_entry_safe(rx, rx_tmp, &stream->rx_queue, list) {
		list_del(&rx->list);
		kfree(rx->data);
		kfree(rx);
	}

	kfree(stream->rx_asm_buf);

	/*
	 * NOT kfree(). odl_tb5_stream_lookup() walks the stream hash inside
	 * rcu_read_lock() and dereferences the object (stream->id, and the
	 * refcount itself) *before* kref_get_unless_zero() can tell it the
	 * stream is dead. kref_get_unless_zero() stops a dead refcount being
	 * resurrected; it does nothing to stop the memory being freed under a
	 * reader that is mid-walk. Removing with hash_del_rcu() and then
	 * freeing immediately is a use-after-free.
	 *
	 * kfree_rcu() holds the allocation until every reader that could still
	 * be walking the chain has left, which is what hash_del_rcu() promised.
	 */
	kfree_rcu(stream, rcu);
}

struct odl_tb5_stream *odl_tb5_stream_create(struct odl_tb5_device *dev,
					     struct odl_tb5_file_ctx *owner,
					     u8 filter_id)
{
	struct odl_tb5_stream *stream;
	int id;

	stream = kzalloc(sizeof(*stream), GFP_KERNEL);
	if (!stream)
		return ERR_PTR(-ENOMEM);

	if (filter_id == 0) {
		id = ida_alloc_range(&dev->stream_ida, 20, 255, GFP_KERNEL);
		if (id < 0) {
			kfree(stream);
			return ERR_PTR(id);
		}
		stream->id = (u8)id;
	} else {
		if (filter_id < 1) {
			kfree(stream);
			return ERR_PTR(-EINVAL);
		}
		id = ida_alloc_range(&dev->stream_ida,
				     filter_id, filter_id, GFP_KERNEL);
		if (id < 0) {
			kfree(stream);
			return ERR_PTR(id);
		}
		stream->id = filter_id;
	}

	stream->dev = dev;
	stream->owner = owner;
	/* Pin the stream to one TX path: all frames of a stream travel on
	 * the same ring, so per-stream ordering needs no reorder logic. */
	stream->path_idx = stream->id % max_t(int, dev->tx_active_paths, 1);
	INIT_LIST_HEAD(&stream->tx_queue);
	spin_lock_init(&stream->tx_lock);
	stream->tx_queue_len = 0;
	stream->tx_queue_max = 64;
	atomic_set(&stream->tx_completed, 0);
	atomic_set(&stream->tx_in_flight, 0);
	init_waitqueue_head(&stream->tx_waitq);

	INIT_LIST_HEAD(&stream->rx_queue);
	spin_lock_init(&stream->rx_lock);
	stream->rx_queue_len = 0;
	/* A single ~1 MiB message needs about 264 frames. The old limit of
	 * 256 silently dropped complete messages when a busy receiver fell
	 * behind, desynchronizing the transport and hanging its consumer
	 * (RCCL/vLLM). Interim: the real fix is credit-based E2E flow
	 * control. */
	stream->rx_queue_max = 65536;
	stream->rx_asm_next_frag = 0;
	stream->rx_asm_bad = false;
	atomic_set(&stream->rx_frag_drops, 0);
	atomic_set(&stream->rx_complete, 0);
	init_waitqueue_head(&stream->rx_waitq);

	kref_init(&stream->refcount);

	mutex_lock(&dev->stream_lock);
	hash_add_rcu(dev->streams, &stream->node, stream->id);
	mutex_unlock(&dev->stream_lock);

	if (owner) {
		spin_lock(&owner->lock);
		list_add(&stream->owner_list, &owner->streams);
		spin_unlock(&owner->lock);
	}

	/* RX pool repost is armed lazily on the first host stream_recv
	 * (odl_tb5_rx_arm), NOT here: a dmabuf-only QP opens a stream for its
	 * qp_num but must leave the RX ring empty so its dmabuf frames receive
	 * the data.  See odl_tb5_rx_arm for the full rationale. */

	pr_info("odl_tb5: stream %u created (owner=%px)\n",
		stream->id, owner);
	return stream;
}

void odl_tb5_stream_destroy(struct odl_tb5_stream *stream)
{
	struct odl_tb5_device *dev = stream->dev;

	pr_info("odl_tb5: stream %u destroying\n", stream->id);

	/*
	 * Order matters. Publish the shutdown state first, then unpublish the
	 * stream so no new waiter can find it, then release everyone already
	 * waiting. A waiter that evaluates its condition after this store
	 * returns immediately instead of sleeping on a dead stream.
	 */
	WRITE_ONCE(stream->dying, true);

	mutex_lock(&dev->stream_lock);

	hash_del_rcu(&stream->node);
	mutex_unlock(&dev->stream_lock);

	if (stream->owner) {
		spin_lock(&stream->owner->lock);
		list_del(&stream->owner_list);
		spin_unlock(&stream->owner->lock);
	}

	/*
	 * Without this, a thread parked in a blocking receive is never released:
	 * the only other wake site is data arrival, and no data is coming. The
	 * caller then blocks forever joining it. Closing the fd does not help
	 * either — release() cannot run while a thread is inside an ioctl on
	 * that fd, so the stream is not even destroyed until the process exits.
	 */
	wake_up_interruptible_all(&stream->rx_waitq);
	wake_up_interruptible_all(&stream->tx_waitq);

	ida_free(&dev->stream_ida, stream->id);
	kref_put(&stream->refcount, odl_tb5_stream_free);
}

void odl_tb5_stream_put(struct odl_tb5_stream *stream)
{
	kref_put(&stream->refcount, odl_tb5_stream_free);
}

void odl_tb5_streams_destroy_all(struct odl_tb5_device *dev)
{
	struct odl_tb5_stream *stream;
	struct hlist_node *tmp;
	int bkt;

	mutex_lock(&dev->stream_lock);
	hash_for_each_safe(dev->streams, bkt, tmp, stream, node) {

		WRITE_ONCE(stream->dying, true);

		hash_del_rcu(&stream->node);

		if (stream->owner) {
			spin_lock(&stream->owner->lock);
			list_del(&stream->owner_list);
			spin_unlock(&stream->owner->lock);
		}

		/* Same reasoning as odl_tb5_stream_destroy(): device teardown
		 * must not leave a caller blocked on a stream it just removed. */
		wake_up_interruptible_all(&stream->rx_waitq);
		wake_up_interruptible_all(&stream->tx_waitq);

		ida_free(&dev->stream_ida, stream->id);
		kref_put(&stream->refcount, odl_tb5_stream_free);
	}
	mutex_unlock(&dev->stream_lock);
}

struct odl_tb5_stream *odl_tb5_stream_lookup(struct odl_tb5_device *dev,
					     u8 stream_id)
{
	struct odl_tb5_stream *stream;

	rcu_read_lock();
	hash_for_each_possible_rcu(dev->streams, stream, node, stream_id) {
		if (stream->id == stream_id) {
			/* Stream may be concurrently torn down (hash_del_rcu
			 * in stream_destroy); only take a ref if still alive. */
			if (!kref_get_unless_zero(&stream->refcount))
				break;
			rcu_read_unlock();
			return stream;
		}
	}
	rcu_read_unlock();
	return NULL;
}

/* ══════════════════════════════════════════════════════════════════════
 * Stream TX Path — Adaptive Latency / Throughput Mode
 * ══════════════════════════════════════════════════════════════════════ */

/*
 * Evaluate whether to use latency or throughput TX mode.
 *
 * Throughput mode uses pre-allocated 256KB batch buffers for reduced
 * per-frame overhead (fewer pool locks, larger copy_from_user calls).
 * Latency mode uses per-frame pool slots for minimal single-frame delay.
 */
static enum odl_tb5_tx_mode
odl_tb5_evaluate_tx_mode(struct odl_tb5_device *dev, size_t msg_len)
{
	unsigned int pool_used;
	unsigned int nframes;

	/* Batch pool not available — latency only */
	if (!dev->batch_pool.bufs[0].virt)
		return ODL_TB5_TX_LATENCY;

	/* Large messages always use throughput mode */
	if (msg_len > ODL_TB5_THROUGHPUT_THRESH) {
		dev->tx_adaptive.consecutive_low = 0;
		dev->tx_adaptive.mode = ODL_TB5_TX_THROUGHPUT;
		return ODL_TB5_TX_THROUGHPUT;
	}

	pool_used = dev->frame_pool.size - dev->frame_pool.free_count;
	nframes = DIV_ROUND_UP(msg_len, ODL_TB5_STREAM_PAYLOAD_MAX);

	if (dev->tx_adaptive.mode == ODL_TB5_TX_LATENCY) {
		/* Transition up: load + new msg exceeds high watermark */
		if (pool_used + nframes > dev->tx_adaptive.high_watermark) {
			dev->tx_adaptive.consecutive_low = 0;
			dev->tx_adaptive.mode = ODL_TB5_TX_THROUGHPUT;
			pr_debug("odl_tb5: TX mode → THROUGHPUT "
				 "(pool_used=%u nframes=%u)\n",
				 pool_used, nframes);
		}
	} else {
		/* Transition down: load below low watermark for N sends */
		if (pool_used < dev->tx_adaptive.low_watermark) {
			if (++dev->tx_adaptive.consecutive_low >=
			    ODL_TB5_MODE_HYSTERESIS) {
				dev->tx_adaptive.mode = ODL_TB5_TX_LATENCY;
				dev->tx_adaptive.consecutive_low = 0;
				pr_debug("odl_tb5: TX mode → LATENCY "
					 "(pool_used=%u)\n", pool_used);
			}
		} else {
			dev->tx_adaptive.consecutive_low = 0;
		}
	}

	return dev->tx_adaptive.mode;
}

/*
 * Latency-mode send — per-frame pool slots with backpressure.
 *
 * Each frame-sized chunk is copied directly from userspace into a DMA
 * pool slot, submitted to the NHI TX ring, and the slot is recycled by
 * the TX callback.  If the pool runs low, we block until a TX callback
 * frees a slot — this provides natural flow control.
 */
static int odl_tb5_stream_send_latency(struct odl_tb5_stream *stream,
					u8 dst_id,
					const void __user *data,
					size_t len)
{
	u16 frag_idx = 0;
	struct odl_tb5_device *dev = stream->dev;
	struct odl_tb5_frame_pool *pool = &dev->frame_pool;
	struct odl_tb5_tx_msg *msg;
	struct odl_tb5_frame_slot *slot;
	struct odl_tb5_stream_hdr *hdr;
	int tp = odl_tb5_stream_tx_path(stream);
	long ret;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	msg->data = NULL;
	msg->dst_id = dst_id;
	msg->len = len;
	msg->sent = 0;
	atomic_set(&msg->frames_pending, 0);
	msg->done = false;
	msg->stream = stream;
	INIT_LIST_HEAD(&msg->list);

	atomic_inc(&stream->tx_in_flight);

	while (msg->sent < len) {
		size_t remain = len - msg->sent;
		size_t payload = min_t(size_t, remain,
				       ODL_TB5_STREAM_PAYLOAD_MAX);
		bool first = (msg->sent == 0);
		bool last  = (msg->sent + payload == len);

		ret = wait_event_interruptible_timeout(pool->avail_waitq,
			pool->free_count > ODL_TB5_TX_POOL_RESERVE,
			msecs_to_jiffies(5000));
		if (ret <= 0) {
			if (ret == 0)
				ret = -ETIMEDOUT;
			goto wait_pending;
		}

		slot = odl_tb5_frame_pool_get(pool);
		if (!slot)
			continue;

		hdr = slot->virt;
		hdr->src_id = stream->id;
		hdr->dst_id = dst_id;
		hdr->reserved = 0;
		hdr->frag_idx = cpu_to_le16(frag_idx++);
		if (first && last)
			hdr->flags = ODL_TB5_SHDR_F_SINGLE;
		else if (first)
			hdr->flags = ODL_TB5_SHDR_F_MSG_START;
		else if (last)
			hdr->flags = ODL_TB5_SHDR_F_MSG_END;
		else
			hdr->flags = 0;
		hdr->payload_len = cpu_to_le16(payload);

		if (copy_from_user(slot->virt + ODL_TB5_STREAM_HDR_SIZE,
				   data + msg->sent, payload)) {
			odl_tb5_frame_pool_put(pool, slot);
			ret = -EFAULT;
			goto wait_pending;
		}

		slot->frame.size = ODL_TB5_STREAM_HDR_SIZE + payload;
		slot->frame.sof  = ODL_TB5_PDF_SOF_DATA;
		slot->frame.eof  = ODL_TB5_PDF_EOF_DATA;
		slot->frame.callback = odl_tb5_tx_callback;
		slot->tx_msg = msg;

		atomic_inc(&msg->frames_pending);
		msg->sent += payload;

		if (tb_ring_tx(dev->paths[tp].tx.ring, &slot->frame) < 0) {
			atomic_dec(&msg->frames_pending);
			msg->sent -= payload;
			odl_tb5_frame_pool_put(pool, slot);
			ret = -EIO;
			goto wait_pending;
		}

		ODL_STAT_INC(dev, tx_frames_submitted);
		atomic64_inc(&dev->stats.path_tx_frames[tp]);
		odl_tb5_tx_submitted(dev);
	}

	return 0;

wait_pending:
	if (atomic_read(&msg->frames_pending) > 0) {
		wait_event_interruptible_timeout(stream->tx_waitq,
			atomic_read(&msg->frames_pending) == 0,
			msecs_to_jiffies(1000));
	}
	if (atomic_read(&msg->frames_pending) == 0) {
		atomic_dec(&stream->tx_in_flight);
		wake_up_interruptible(&stream->tx_waitq);
		kfree(msg);
	}
	return (int)ret;
}

/*
 * Throughput-mode send — uses pre-allocated 256KB contiguous DMA batch
 * buffers to reduce per-frame overhead.  Each batch holds up to 64
 * frames (64 × 4091 = 261,824 bytes of payload).  Benefits:
 *   - 1 batch buffer allocation vs. 64 pool slot allocations
 *   - Copy data in larger chunks (vs. 4KB per frame)
 *   - Frames are pre-staged before submission
 */
static int odl_tb5_stream_send_throughput(struct odl_tb5_stream *stream,
					   u8 dst_id,
					   const void __user *data,
					   size_t len)
{
	u16 frag_idx = 0;
	struct odl_tb5_device *dev = stream->dev;
	struct odl_tb5_batch_pool *bpool = &dev->batch_pool;
	struct odl_tb5_tx_msg *msg;
	size_t total_sent = 0;
	int tp = odl_tb5_stream_tx_path(stream);
	long ret;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	msg->data = NULL;
	msg->dst_id = dst_id;
	msg->len = len;
	msg->sent = 0;
	atomic_set(&msg->frames_pending, 0);
	msg->done = false;
	msg->stream = stream;
	INIT_LIST_HEAD(&msg->list);

	atomic_inc(&stream->tx_in_flight);

	while (total_sent < len) {
		struct odl_tb5_batch_buf *batch;
		size_t batch_payload_cap;
		size_t batch_payload;
		int nframes, i;

		/* Wait for a free batch buffer */
		ret = wait_event_interruptible_timeout(bpool->avail_waitq,
			bpool->free_count > 0,
			msecs_to_jiffies(5000));
		if (ret <= 0) {
			pr_warn_ratelimited("odl_tb5: throughput TX stalled waiting for batch buf (free=%d ret=%ld sent=%zu/%zu)\n",
					    bpool->free_count, ret,
					    total_sent, len);
			if (ret == 0)
				ret = -ETIMEDOUT;
			goto wait_pending;
		}

		batch = odl_tb5_batch_pool_get(bpool);
		if (!batch)
			continue; /* spurious wakeup */

		/* How much user data fits in one batch buffer? */
		batch_payload_cap = (size_t)ODL_TB5_BATCH_FRAMES *
				    ODL_TB5_STREAM_PAYLOAD_MAX;
		batch_payload = min_t(size_t, len - total_sent,
				      batch_payload_cap);
		nframes = DIV_ROUND_UP(batch_payload,
				       ODL_TB5_STREAM_PAYLOAD_MAX);

		/* Fill each frame within the batch buffer */
		for (i = 0; i < nframes; i++) {
			void *frame_base = batch->virt +
					   ((size_t)i * ODL_TB5_FRAME_SIZE);
			struct odl_tb5_stream_hdr *hdr = frame_base;
			size_t offset = (size_t)i * ODL_TB5_STREAM_PAYLOAD_MAX;
			size_t payload = min_t(size_t,
					       batch_payload - offset,
					       ODL_TB5_STREAM_PAYLOAD_MAX);
			bool first = (total_sent == 0 && i == 0);
			bool last  = (total_sent + offset + payload == len);

			/* Stream header */
			hdr->src_id = stream->id;
			hdr->dst_id = dst_id;
			hdr->reserved = 0;
			hdr->frag_idx = cpu_to_le16(frag_idx++);
			if (first && last)
				hdr->flags = ODL_TB5_SHDR_F_SINGLE;
			else if (first)
				hdr->flags = ODL_TB5_SHDR_F_MSG_START;
			else if (last)
				hdr->flags = ODL_TB5_SHDR_F_MSG_END;
			else
				hdr->flags = 0;
			hdr->payload_len = cpu_to_le16(payload);

			/* Copy payload from userspace */
			if (copy_from_user(frame_base +
					   ODL_TB5_STREAM_HDR_SIZE,
					   data + total_sent + offset,
					   payload)) {
				odl_tb5_batch_pool_put(bpool, batch);
				ret = -EFAULT;
				goto wait_pending;
			}

			/* Set up ring descriptor */
			batch->frames[i].buffer_phy = batch->phys +
				((size_t)i * ODL_TB5_FRAME_SIZE);
			batch->frames[i].size = ODL_TB5_STREAM_HDR_SIZE +
						payload;
			batch->frames[i].sof = ODL_TB5_PDF_SOF_DATA;
			batch->frames[i].eof = ODL_TB5_PDF_EOF_DATA;
			batch->frames[i].callback =
				odl_tb5_tx_batch_callback;
		}

		/* Arm completion tracking before submission */
		batch->tx_msg = msg;
		batch->total_frames = nframes;
		atomic_set(&batch->frames_pending, nframes);
		atomic_add(nframes, &msg->frames_pending);
		msg->sent += batch_payload;

		/* Submit all frames in this batch to the ring */
		for (i = 0; i < nframes; i++) {
			if (tb_ring_tx(dev->paths[tp].tx.ring,
				       &batch->frames[i]) < 0) {
				int unsub = nframes - i;

				atomic_sub(unsub, &batch->frames_pending);
				atomic_sub(unsub, &msg->frames_pending);
				msg->sent -= (size_t)unsub *
					     ODL_TB5_STREAM_PAYLOAD_MAX;
				batch->total_frames = i;
				if (i == 0)
					odl_tb5_batch_pool_put(bpool,
							       batch);
				ret = -EIO;
				goto wait_pending;
			}

			ODL_STAT_INC(dev, tx_frames_submitted);
			atomic64_inc(&dev->stats.path_tx_frames[tp]);
			odl_tb5_tx_submitted(dev);
		}

		total_sent += batch_payload;
	}

	return 0;

wait_pending:
	if (atomic_read(&msg->frames_pending) > 0) {
		wait_event_interruptible_timeout(stream->tx_waitq,
			atomic_read(&msg->frames_pending) == 0,
			msecs_to_jiffies(1000));
	}
	if (atomic_read(&msg->frames_pending) == 0) {
		atomic_dec(&stream->tx_in_flight);
		wake_up_interruptible(&stream->tx_waitq);
		kfree(msg);
	}
	return (int)ret;
}

/*
 * Non-blocking availability checks (for poll/epoll + async ioctl).
 * Returns true if a send/recv would not block.
 */
extern unsigned int odl_busy_poll_us;

/* Optional bounded busy-poll for an RX completion before sleeping.  The RX
 * softirq (odl_tb5_rx_callback) increments rx_complete on another CPU, so a
 * spinning reader sees it within cache-coherency latency and skips the
 * context-switch wake (~10-15 us on this box).  Bounded + falls back to
 * wait_event, so it never hangs.  Off unless odl_busy_poll_us > 0.
 * (upstream PR #21) */
static inline void odl_tb5_rx_busy_poll(struct odl_tb5_stream *stream)
{
	ktime_t deadline;

	if (!odl_busy_poll_us || atomic_read(&stream->rx_complete) > 0)
		return;
	deadline = ktime_add_ns(ktime_get(), (u64)odl_busy_poll_us * 1000);
	while (atomic_read(&stream->rx_complete) == 0) {
		if (ktime_after(ktime_get(), deadline))
			break;
		cpu_relax();
	}
}

bool odl_tb5_stream_can_send(struct odl_tb5_stream *stream)
{
	struct odl_tb5_device *dev = stream->dev;
	struct odl_tb5_frame_pool *pool = &dev->frame_pool;
	bool ok;

	/* Can send if we have frames available and state is ready */
	ok = dev->state == ODL_TB5_STATE_READY &&
	     pool && pool->free_count > ODL_TB5_TX_POOL_RESERVE;

	if (!ok) {
		/* rx_target/rx_posted are per-path here; report the control
		 * path (0), which is the one stream traffic is pinned to. */
		const struct odl_tb5_path *p0 = &dev->paths[0];

		pr_debug("odl_tb5: can_send=0 state=%d free=%d reserve=%d rx_target=%d rx_posted=%d\n",
			 dev->state, pool ? pool->free_count : -1,
			 ODL_TB5_TX_POOL_RESERVE, p0->rx_target,
			 atomic_read(&p0->rx_posted));
	}
	return ok;
}

bool odl_tb5_stream_can_recv(struct odl_tb5_stream *stream)
{
	/* Can recv if data is already in the RX queue */
	return atomic_read(&stream->rx_complete) > 0;
}

/*
 * Stream send — adaptive dispatcher.
 *
 * Evaluates TX mode (latency vs throughput) based on message size and
 * current load, then dispatches to the appropriate send path.
 */
int odl_tb5_stream_send(struct odl_tb5_stream *stream,
			u8 dst_id, const void __user *data, size_t len)
{
	struct odl_tb5_device *dev = stream->dev;
	enum odl_tb5_tx_mode mode;

	ODL_STAT_INC(dev, tx_send_calls);

	if (dev->state != ODL_TB5_STATE_READY)
		return -ENOTCONN;

	if (len == 0 || len > (size_t)ODL_TB5_STREAM_PAYLOAD_MAX * 4096)
		return -EINVAL;

	ODL_STAT_ADD(dev, tx_bytes_submitted, len);

	mode = odl_tb5_evaluate_tx_mode(dev, len);

	if (mode == ODL_TB5_TX_THROUGHPUT)
		return odl_tb5_stream_send_throughput(stream, dst_id,
						      data, len);
	return odl_tb5_stream_send_latency(stream, dst_id, data, len);
}

void odl_tb5_tx_drain_work_fn(struct work_struct *work)
{
	struct odl_tb5_device *dev =
		container_of(work, struct odl_tb5_device, tx_drain_work);
	struct odl_tb5_stream *stream;
	struct odl_tb5_tx_msg *msg;
	struct odl_tb5_frame_slot *slot;
	struct odl_tb5_stream_hdr *hdr;
	unsigned long flags;
	int bkt;
	bool did_work;

	do {
		did_work = false;

		rcu_read_lock();
		hash_for_each(dev->streams, bkt, stream, node) {
			int tp = odl_tb5_stream_tx_path(stream);

			spin_lock_irqsave(&stream->tx_lock, flags);
			msg = list_first_entry_or_null(&stream->tx_queue,
						       struct odl_tb5_tx_msg,
						       list);
			if (!msg || msg->sent >= msg->len) {
				spin_unlock_irqrestore(&stream->tx_lock, flags);
				continue;
			}
			spin_unlock_irqrestore(&stream->tx_lock, flags);

			slot = odl_tb5_frame_pool_get(&dev->frame_pool);
			if (!slot)
				goto out;

			/* Build frame: 5-byte stream header + payload */
			hdr = slot->virt;
			hdr->src_id = stream->id;
			hdr->dst_id = msg->dst_id;

			{
				size_t remain = msg->len - msg->sent;
				size_t payload = min_t(size_t, remain,
						       ODL_TB5_STREAM_PAYLOAD_MAX);
				bool first = (msg->sent == 0);
				bool last = (msg->sent + payload == msg->len);

				if (first && last)
					hdr->flags = ODL_TB5_SHDR_F_SINGLE;
				else if (first)
					hdr->flags = ODL_TB5_SHDR_F_MSG_START;
				else if (last)
					hdr->flags = ODL_TB5_SHDR_F_MSG_END;
				else
					hdr->flags = 0;

				hdr->payload_len = cpu_to_le16(payload);
				memcpy(slot->virt + ODL_TB5_STREAM_HDR_SIZE,
				       msg->data + msg->sent, payload);

				slot->frame.size = ODL_TB5_STREAM_HDR_SIZE + payload;
				slot->frame.sof = ODL_TB5_PDF_SOF_DATA;
				slot->frame.eof = ODL_TB5_PDF_EOF_DATA;
				slot->frame.callback = odl_tb5_tx_callback;
				slot->tx_msg = msg;

				msg->sent += payload;
				atomic_inc(&msg->frames_pending);
			}

			if (tb_ring_tx(dev->paths[tp].tx.ring, &slot->frame) < 0) {
				pr_warn("odl_tb5: tb_ring_tx failed for "
					"stream %u (sent=%zu/%zu)\n",
					stream->id, msg->sent, msg->len);
				msg->sent -= le16_to_cpu(hdr->payload_len);
				atomic_dec(&msg->frames_pending);
				odl_tb5_frame_pool_put(&dev->frame_pool, slot);
				/* Wake waiter so timeout can fire */
				wake_up_interruptible(&stream->tx_waitq);
				goto out;
			}

			ODL_STAT_INC(dev, tx_frames_submitted);
			atomic64_inc(&dev->stats.path_tx_frames[tp]);
			odl_tb5_tx_submitted(dev);
			did_work = true;
		}
out:
		rcu_read_unlock();
	} while (did_work);
}

int odl_tb5_stream_wait_tx(struct odl_tb5_stream *stream, u32 timeout_ms)
{
	long ret;

	if (timeout_ms == 0) {
		ret = wait_event_interruptible(stream->tx_waitq,
			atomic_read(&stream->tx_in_flight) == 0);
	} else {
		ret = wait_event_interruptible_timeout(stream->tx_waitq,
			atomic_read(&stream->tx_in_flight) == 0,
			msecs_to_jiffies(timeout_ms));
		if (ret == 0)
			return -ETIMEDOUT;
	}

	return ret < 0 ? -EINTR : 0;
}

/* ══════════════════════════════════════════════════════════════════════
 * Stream RX Path
 * ══════════════════════════════════════════════════════════════════════ */

/* Arm the stream RX pool frames the first time a host stream recv is
 * ATTEMPTED.  Called from the STREAM_RECV / STREAM_WAIT_RX ioctl handlers
 * (odl_tb5_chardev.c) — deliberately BEFORE their O_NONBLOCK/can_recv check,
 * because the verbs provider opens the device non-blocking: its recv worker
 * gets -EAGAIN and never reaches odl_tb5_stream_recv() below, so arming inside
 * that function would never fire (pool never armed -> no data -> EAGAIN
 * forever -> host RX hang).
 *
 * Deliberately NOT armed in odl_tb5_stream_create: a QP that only does
 * zero-copy dmabuf recv opens a stream (for its qp_num) but issues only
 * STREAM_RECV_DMABUF (submit_rx_dmabuf), never STREAM_RECV, so its RX ring
 * stays empty and the posted dmabuf frames are the sole consumers — matching
	 * the synchronous dmabuf path.  Arming at open instead filled the shared
 * paths[0].rx ring with auto-reposting pool frames that swallowed the dmabuf
 * payload.  Lock-free: a redundant concurrent call just re-runs rx_repost,
 * which caps at rx_target, so at worst the pool is topped up twice. */
void odl_tb5_rx_arm(struct odl_tb5_device *dev)
{
	int per_path;

	if (dev->paths[0].rx_target != 0 || !dev->frame_pool.slots)
		return;

	/* The entire pool / RDMA-header receive window lives on the control
	 * path (0).  With multiple paths, synchronous dmabuf traffic uses the
	 * reserved data paths.  With one path, its RX call temporarily quiesces
	 * auto-repost through dmabuf_rx_active, so ordinary stream frames and
	 * private dmabuf staging frames do not share the FIFO.
	 * We do NOT arm pool on any path other than 0.  Cap the window to half
	 * the RX ring depth (design intent: never fully saturate the ring) so
	 * a large pool can't overflow a small ring.  In loopback (ring_size 0)
	 * leave the window uncapped. */
	per_path = dev->frame_pool.size / 2;
	if (dev->paths[0].rx.ring_size > 0) {
		int rx_cap = dev->paths[0].rx.ring_size / 2;

		if (per_path > rx_cap)
			per_path = rx_cap;
	}
	dev->paths[0].rx_target = per_path;
	odl_tb5_rx_repost(dev, 0);
	pr_info("odl_tb5: RX pool armed on control path (target=%d)\n",
		dev->paths[0].rx_target);
}

int odl_tb5_stream_recv(struct odl_tb5_stream *stream,
			void __user *buf, size_t buf_len,
			u8 *src_id, u32 *actual_len)
{
	struct odl_tb5_rx_msg *msg;
	unsigned long flags;
	int ret = 0;

	/* Wait for a complete assembled message, or for teardown. Testing
	 * ->dying is what makes closing a stream able to cancel a blocked
	 * receive; report it distinctly so the caller can tell "shutting down"
	 * apart from "spurious wake, queue empty" (-EAGAIN below). */
	odl_tb5_rx_busy_poll(stream);
	ret = wait_event_interruptible(stream->rx_waitq,
		atomic_read(&stream->rx_complete) > 0 ||
		READ_ONCE(stream->dying));
	if (ret)
		return -EINTR;
	if (READ_ONCE(stream->dying) && atomic_read(&stream->rx_complete) <= 0)
		return -ESHUTDOWN;

	/* Dequeue one complete message */
	spin_lock_irqsave(&stream->rx_lock, flags);
	msg = list_first_entry_or_null(&stream->rx_queue,
				       struct odl_tb5_rx_msg, list);
	if (msg) {
		list_del(&msg->list);
		stream->rx_queue_len--;
	}
	spin_unlock_irqrestore(&stream->rx_lock, flags);

	if (!msg)
		return -EAGAIN;

	atomic_dec(&stream->rx_complete);

	*src_id = msg->src_id;
	*actual_len = min_t(size_t, msg->len, buf_len);
	if (copy_to_user(buf, msg->data, *actual_len))
		ret = -EFAULT;

	kfree(msg->data);
	kfree(msg);
	return ret;
}

int odl_tb5_stream_wait_rx(struct odl_tb5_stream *stream, u32 timeout_ms)
{
	long ret;

	odl_tb5_rx_busy_poll(stream);

	if (timeout_ms == 0) {
		ret = wait_event_interruptible(stream->rx_waitq,
			atomic_read(&stream->rx_complete) > 0 ||
			READ_ONCE(stream->dying));
	} else {
		ret = wait_event_interruptible_timeout(stream->rx_waitq,
			atomic_read(&stream->rx_complete) > 0 ||
			READ_ONCE(stream->dying),
			msecs_to_jiffies(timeout_ms));
		if (ret == 0)
			return -ETIMEDOUT;
	}

	if (ret < 0)
		return -EINTR;

	/* Woken by teardown rather than by data. */
	if (READ_ONCE(stream->dying) && atomic_read(&stream->rx_complete) <= 0)
		return -ESHUTDOWN;

	return 0;
}

/* ══════════════════════════════════════════════════════════════════════
 * RX Repost — keep RX ring filled with pool frames
 * ══════════════════════════════════════════════════════════════════════ */

void odl_tb5_rx_repost(struct odl_tb5_device *dev, int idx)
{
	struct odl_tb5_path *path = &dev->paths[idx];
	int target;
	unsigned long flags;

	/* Pair with the single-path dmabuf ownership handoff.  Admission is
	 * serialized under the ring-context lock; dmabuf raises its guard under
	 * the same lock and then waits for already-admitted refillers. */
	spin_lock_irqsave(&path->rx.lock, flags);

	/* dmabuf ring separation: single-path fallback.  When dmabuf shares
	 * the control path (ndp==0 in odl_tb5_submit_dmabuf, which is the
	 * negotiated-before == 1 cap OR symmetric negotiated_paths<2), a
	 * dmabuf RX is in flight and owns the RX ring; keep it empty of pool
	 * frames until the transfer completes.  The gate is dmabuf_rx_active,
	 * NOT negotiated_paths: dmabuf_paths=1 caps nps to 1 while the link
	 * still negotiated 2 paths, so a negotiated_paths<2 test here wrongly
	 * lets the pool repost onto a ring the dmabuf transfer owns — pool
	 * frames then compete with the dmabuf's own ring slots and can starve
	 * it by one frame, hanging the transfer (the F2 hang).  In multi-path
	 * mode dmabuf lives on separate rings (1..ndp) and dmabuf_rx_active is
	 * never raised, so this guard does not fire and the pool keeps the
	 * control path topped up. */
	if (idx == 0 && atomic_read(&dev->dmabuf_rx_active) > 0) {
		spin_unlock_irqrestore(&path->rx.lock, flags);
		return;
	}
	/* One refiller is enough to reach target.  Excluding additional
	 * refillers also makes the rx_posted check-and-post sequence exact. */
	if (atomic_read(&path->rx_reposting) > 0) {
		/* Do not lose the demand: the owner may currently be leaving after
		 * a full-ring/pool-empty result just as this callback frees a slot. */
		path->rx_repost_pending = true;
		spin_unlock_irqrestore(&path->rx.lock, flags);
		return;
	}
	atomic_set(&path->rx_reposting, 1);
	path->rx_repost_pending = false;
	spin_unlock_irqrestore(&path->rx.lock, flags);


refill:
	target = READ_ONCE(path->rx_target);

	/* Re-read rx_posted each iteration: this runs concurrently from
	 * every RX callback and from stream_create, and a stale local
	 * copy lets racing callers each post a full target's worth. */
	while (atomic_read(&path->rx_posted) < target) {
		struct odl_tb5_frame_slot *slot;

		slot = odl_tb5_frame_pool_get(&dev->frame_pool);
		if (!slot) {
			ODL_STAT_INC(dev, rx_repost_pool_empty);
			/*
			 * Pool exhausted: the receive ring stays short of
			 * rx_target and inbound frames will be dropped by the
			 * NHI with no error flag set. Record it — this used to
			 * be a bare break, which is why the loss had no
			 * fingerprint.
			 */
			int posted = atomic_read(&path->rx_posted);
			int shortfall = target - posted;

			atomic_inc(&dev->rx_repost_starved);
			if (shortfall > atomic_read(&dev->rx_repost_short))
				atomic_set(&dev->rx_repost_short, shortfall);
			pr_warn_ratelimited("odl_tb5: RX repost starved: path=%d posted=%d target=%d short=%d pool_free=%d (frames will be dropped unflagged)\n",
					    idx, posted, target, shortfall,
					    dev->frame_pool.free_count);
			break;
		}

		slot->frame.buffer_phy = slot->phys;
		slot->frame.size = 0; /* NHI fills this on RX completion */
		slot->frame.callback = odl_tb5_rx_callback;
		slot->frame.sof = ODL_TB5_PDF_SOF_DATA;
		slot->frame.eof = ODL_TB5_PDF_EOF_DATA;

		if (tb_ring_rx(path->rx.ring, &slot->frame) < 0) {
			ODL_STAT_INC(dev, rx_repost_ring_fail);
			odl_tb5_frame_pool_put(&dev->frame_pool, slot);
			break;
		}

		atomic_inc(&path->rx_posted);
	}

	/* Low-water mark, so a transient dip that never fully starves is still
	 * visible after the fact.  Device-wide across paths: the interesting
	 * question is whether ANY receive ring ran dry. */
	{
		int posted = atomic_read(&path->rx_posted);

		if (posted < atomic_read(&dev->rx_posted_min))
			atomic_set(&dev->rx_posted_min, posted);
	}

	/* Close ownership under the same admission lock.  A completion can
	 * decrement rx_posted and call back into this function between the last
	 * loop test and here; it records pending demand when it observes the
	 * current owner.  Consume that demand before clearing ownership so the
	 * newly freed pool/ring slot gets one event-driven retry. */
	spin_lock_irqsave(&path->rx.lock, flags);
	if (path->rx_repost_pending &&
	    !(idx == 0 && atomic_read(&dev->dmabuf_rx_active) > 0) &&
	    atomic_read(&path->rx_posted) < READ_ONCE(path->rx_target)) {
		/* Consume exactly the demand observed since the last attempt.  If
		 * this retry blocks again it releases unless another callback creates
		 * fresh demand, avoiding an unconditional full-ring spin. */
		path->rx_repost_pending = false;
		spin_unlock_irqrestore(&path->rx.lock, flags);
		goto refill;
	}
	path->rx_repost_pending = false;
	atomic_set(&path->rx_reposting, 0);
	spin_unlock_irqrestore(&path->rx.lock, flags);
	wake_up_all(&path->rx_repost_waitq);
}
