// SPDX-License-Identifier: GPL-2.0-only
/*
 * OdinLink — Fake Peer for Testing Without a Cable
 *
 * When you pass loopback=1 to the module, this file creates pretend
 * Thunderbolt devices. No NHI hardware needed — data just loops back
 * inside your own machine via memcpy.
 *
 * Use: sudo insmod driver/odl_tb5.ko loopback=1
 * Then: /dev/odl_tb5_0 appears, immediately in READY state.
 *
 * What's fake:
 *   - No real DMA — data moves at memcpy speed (not 80 Gbps)
 *   - No real interrupts — completions are instant
 *   - The "peer" is just another software buffer on the same machine
 *
 * What's real:
 *   - The full ioctl API works (streams, legacy, mmap, poll)
 *   - Good for testing apps, protocol logic, and verbs provider
 *   - Bad for benchmarking
 */

#include "odl_tb5_core.h"
#include <linux/slab.h>
#include <linux/vmalloc.h>

/* ── Loopback per-stream state ──────────────────────────────────────── */

#define LB_STREAM_MAX 256
#define LB_QUEUE_DEPTH 512

struct lb_msg {
	struct list_head  list;
	void             *data;
	uint32_t          len;
	uint8_t           src_id;
};

struct lb_stream {
	bool              active;
	struct list_head  rx_queue;
	spinlock_t        rx_lock;
	wait_queue_head_t rx_wait;
	int               rx_count;
};

/* ── Loopback device instance ───────────────────────────────────────── */

struct lb_device {
	int               index;
	void             *buf;
	size_t            buf_size;
	size_t            alloc_size;
	struct lb_stream  streams[LB_STREAM_MAX];
	struct mutex      stream_lock;
	unsigned long     stream_bitmap[BITS_TO_LONGS(LB_STREAM_MAX)];
};

/* ── Create / Destroy ───────────────────────────────────────────────── */

static struct odl_tb5_device *lb_create(int index)
{
	struct odl_tb5_device *dev;
	struct lb_device *lb;
	int ret;
	int i;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return ERR_PTR(-ENOMEM);

	dev->index = index;
	/* Loopback is always single-path — no DMA rings to stripe over. */
	dev->num_paths = 1;
	dev->remote_path_count = 1;
	dev->negotiated_paths = 1;
	dev->tx_active_paths = 1;
	dev->paths[0].tx.dev = dev;
	dev->paths[0].rx.dev = dev;
	dev->state = ODL_TB5_STATE_DISCONNECTED;
	mutex_init(&dev->state_lock);
	init_waitqueue_head(&dev->state_waitq);
	hash_init(dev->streams);
	ida_init(&dev->stream_ida);
	mutex_init(&dev->stream_lock);
	atomic_set(&dev->open_count, 0);
	atomic_set(&dev->removing, 0);

	/* Allocate one page-aligned backing store for all four mmap buffers. */
	lb = kzalloc(sizeof(*lb), GFP_KERNEL);
	if (!lb) {
		ret = -ENOMEM;
		goto err_free;
	}

	lb->index = index;
	lb->buf_size = ODL_TB5_FRAME_SIZE * 4096; /* 16 MB per legacy buffer */
	lb->alloc_size = lb->buf_size * ODL_TB5_NUM_BUFFERS * 2;
	lb->buf = vmalloc_user(lb->alloc_size);
	if (!lb->buf) {
		ret = -ENOMEM;
		goto err_lb;
	}

	for (i = 0; i < ODL_TB5_NUM_BUFFERS; i++) {
		dev->paths[0].tx.bufs[i].virt =
			(char *)lb->buf + i * lb->buf_size;
		dev->paths[0].tx.bufs[i].size = lb->buf_size;
		dev->paths[0].rx.bufs[i].virt = (char *)lb->buf +
			(ODL_TB5_NUM_BUFFERS + i) * lb->buf_size;
		dev->paths[0].rx.bufs[i].size = lb->buf_size;
	}

	dev->paths[0].tx.front = 0;
	dev->paths[0].tx.back = 1;
	dev->paths[0].rx.front = 0;
	dev->paths[0].rx.back = 1;
	spin_lock_init(&dev->paths[0].tx.lock);
	spin_lock_init(&dev->paths[0].rx.lock);
	atomic_set(&dev->paths[0].tx.completed, 0);
	atomic_set(&dev->paths[0].tx.submitted, 0);
	atomic_set(&dev->paths[0].rx.completed, 0);
	atomic_set(&dev->paths[0].rx.submitted, 0);
	init_waitqueue_head(&dev->paths[0].tx.waitq);
	init_waitqueue_head(&dev->paths[0].rx.waitq);

	for (i = 0; i < LB_STREAM_MAX; i++) {
		struct lb_stream *s = &lb->streams[i];
		INIT_LIST_HEAD(&s->rx_queue);
		spin_lock_init(&s->rx_lock);
		init_waitqueue_head(&s->rx_wait);
		s->rx_count = 0;
	}

	mutex_init(&lb->stream_lock);
	dev->loopback_data = lb;

	/* Loopback has no peer handshake, so it is ready before publication. */
	mutex_lock(&dev->state_lock);
	dev->state = ODL_TB5_STATE_READY;
	mutex_unlock(&dev->state_lock);

	/* Publish the device only after mmap and poll state is ready. */
	ret = odl_tb5_chardev_create(dev);
	if (ret)
		goto err_buf;

	pr_info("odl_tb5: loopback device %d ready (%d buffers x %zu MB, "
		"%zu MB total)\n", index, ODL_TB5_NUM_BUFFERS * 2,
		lb->buf_size >> 20, lb->alloc_size >> 20);

	return dev;

err_buf:
	dev->loopback_data = NULL;
	vfree(lb->buf);
err_lb:
	kfree(lb);
err_free:
	kfree(dev);
	return ERR_PTR(ret);
}

static void lb_destroy(struct odl_tb5_device *dev)
{
	struct lb_device *lb = dev->loopback_data;
	if (!lb) return;

	atomic_set(&dev->removing, 1);
	odl_tb5_chardev_destroy(dev);
	vfree(lb->buf);
	mutex_destroy(&lb->stream_lock);
	kfree(lb);
	dev->loopback_data = NULL;

	kfree(dev);
}

/* ── Module hooks ───────────────────────────────────────────────────── */

int odl_loopback_init(void)
{
	if (odl_loopback_count <= 0) return 0;
	if (odl_loopback_count > ODL_TB5_MAX_DEVICES)
		odl_loopback_count = ODL_TB5_MAX_DEVICES;

	for (int i = 0; i < odl_loopback_count; i++) {
		struct odl_tb5_device *dev = lb_create(i);
		if (IS_ERR(dev)) continue;
		mutex_lock(&odl_tb5_devices_lock);
		list_add_tail(&dev->list, &odl_tb5_devices_list);
		mutex_unlock(&odl_tb5_devices_lock);
	}
	return 0;
}

void odl_loopback_exit(void)
{
	struct odl_tb5_device *dev, *tmp;
	mutex_lock(&odl_tb5_devices_lock);
	list_for_each_entry_safe(dev, tmp, &odl_tb5_devices_list, list) {
		if (dev->loopback_data) {
			list_del(&dev->list);
			mutex_unlock(&odl_tb5_devices_lock);
			lb_destroy(dev);
			mutex_lock(&odl_tb5_devices_lock);
		}
	}
	mutex_unlock(&odl_tb5_devices_lock);
}
