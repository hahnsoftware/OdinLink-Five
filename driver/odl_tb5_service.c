// SPDX-License-Identifier: MIT
/*
 * OdinLink — Driver Lifecycle: Load, Probe, Remove
 *
 * What happens when you run `sudo insmod odl_tb5.ko`:
 *   1. Advertises "OdinLink is here" to any Thunderbolt peer on the other
 *      end of the cable (via XDomain property directories).
 *   2. When a peer matches our protocol ID, the kernel calls probe(),
 *      which creates a device struct, allocates DMA rings, and kicks off
 *      the login handshake.
 *   3. remove() tears everything down when the cable is unplugged or the
 *      module is unloaded.
 *
 * Also handles module parameters: ring_size, loopback, protocol, e2e.
 */

#include <linux/debugfs.h>
#include <linux/log2.h>

#include "odl_tb5_core.h"

LIST_HEAD(odl_tb5_devices_list);
DEFINE_MUTEX(odl_tb5_devices_lock);

/* Module-global debugfs root — parent of the per-device stat dirs. */
struct dentry *odl_tb5_debugfs_root;

static DEFINE_IDA(odl_tb5_ida);

unsigned int odl_ring_size = ODL_TB5_RING_SIZE_DEFAULT;
module_param(odl_ring_size, uint, 0444);
MODULE_PARM_DESC(odl_ring_size,
	"NHI ring entries per direction (power-of-2, default 4096 = 16 MB/batch; "
	"use 1024 on iommu=pt hosts where 16 MB contiguous DMA fails)");

int odl_loopback_count = 0;
module_param_named(loopback, odl_loopback_count, int, 0444);
MODULE_PARM_DESC(loopback,
	"Create N software loopback devices (max 16, default 0; no NHI hw needed)");

int odl_protocol_mode = 0;
module_param_named(protocol, odl_protocol_mode, int, 0444);
MODULE_PARM_DESC(protocol,
	"XDomain protocol mode: 0=OdinLink (0x4F4C, default), 1=Apple (0xFA57)");

bool odl_e2e = true;
module_param_named(e2e, odl_e2e, bool, 0444);

bool odl_raw_payload = true;
module_param_named(raw_payload, odl_raw_payload, bool, 0444);
MODULE_PARM_DESC(raw_payload,
	"Allow bounce-free raw-payload dmabuf TX/RX when both peers "
	"advertise support (default=1). The login exchange negotiates "
	"per link; old peers fall back to the legacy framed path.");

/* upstream PR #21: bounded RX busy-poll before sleeping. The RX softirq
 * increments rx_complete on another CPU, so a spinning reader sees it within
 * cache-coherency latency and skips a ~10-15 us context-switch wake. */
unsigned int odl_busy_poll_us = 0;
module_param(odl_busy_poll_us, uint, 0644);
MODULE_PARM_DESC(odl_busy_poll_us,
	"Bounded RX busy-poll window in microseconds before sleeping (0 = off)");

/* Bind at most N XDomain services (0 = unlimited).  With two Thunderbolt
 * cables both peer services sit at route=2 (BUG 2) and the handshake cannot
 * complete; unbinding one afterwards runs the full teardown path, so bind
 * only one from the start instead. */
int odl_max_devices = 0;
module_param_named(max_devices, odl_max_devices, int, 0444);
MODULE_PARM_DESC(max_devices,
	"Bind at most N XDomain services (0 = unlimited; use 1 with two cables)");
static atomic_t odl_bound_count = ATOMIC_INIT(0);
MODULE_PARM_DESC(e2e,
	"Enable end-to-end flow control (default=1). Set 0 for TB3 controllers "
	"that do not support RING_FLAG_E2E.");

unsigned int odl_num_paths = 2;
module_param_named(num_paths, odl_num_paths, uint, 0444);
MODULE_PARM_DESC(num_paths,
	"Number of parallel DMA striping paths per device (1.."
	__stringify(ODL_TB5_MAX_PATHS) ", default 2). The router flow-control "
	"cap is per-path, so multiple paths scale throughput.");

/* Diagnostic/override knob for the zero-copy dmabuf stripe width.  0 (default)
 * means "use negotiated_paths"; a positive value caps the number of paths a
 * single dmabuf transfer is striped over.  Writable so a two-box A/B run can
 * flip 1<->2 without reload (BOTH ends must match, or the stripe desyncs). */
unsigned int odl_dmabuf_paths;
module_param_named(dmabuf_paths, odl_dmabuf_paths, uint, 0644);
MODULE_PARM_DESC(dmabuf_paths,
	"Cap dmabuf (zero-copy verbs/RCCL) stripe width; 0 = use negotiated "
	"paths. Set identically on both peers.");

/* Apple protocol uses its own property key and registers as an alternate
 * service so macOS ThunderboltRDMA can discover us via XDomain matching. */
static struct tb_property_dir *odl_tb5_apple_property_dir;

const uuid_t odl_tb5_proto_uuid =
	UUID_INIT(0x4f444c4e, 0x4b54, 0x4235,
		  0x4f, 0x44, 0x49, 0x4e, 0x4c, 0x49, 0x4e, 0x4b);

static struct tb_property_dir *odl_tb5_property_dir;

static const struct tb_service_id odl_tb5_ids[] = {
	{ TB_SERVICE(ODL_TB5_PROTOCOL_KEY, ODL_TB5_PROTOCOL_ID) },
	{ TB_SERVICE(ODL_TB5_PROTOCOL_KEY_APPLE, ODL_TB5_PROTOCOL_ID_APPLE) },
	{ }
};
MODULE_DEVICE_TABLE(tbsvc, odl_tb5_ids);

/* Allocate the NHI ring pairs plus their DMA buffers.  A ring-stage failure
 * gets one probe at the minimum size: if even that fails, halving cannot tell
 * memory pressure from controller resources owned by another driver.  Only a
 * later DMA-buffer failure uses the full halving ladder.  With iommu=pt the
 * NHI sits in an identity IOMMU domain, so large dma_alloc_coherent requests
 * may need that ladder to find physically contiguous memory. */
static int odl_tb5_probe_alloc_rings_bufs(struct odl_tb5_device *dev)
{
	unsigned int rs = odl_ring_size;
	int ret;

	if (rs < ODL_TB5_RING_SIZE_MIN)
		rs = ODL_TB5_RING_SIZE_MIN;
	if (rs > ODL_TB5_RING_SIZE_MAX)
		rs = ODL_TB5_RING_SIZE_MAX;
	rs = roundup_pow_of_two(rs);

	for (;;) {
		ret = odl_tb5_rings_alloc(dev, rs);
		if (ret) {
			odl_tb5_rings_free(dev);

			if (ret == -EBUSY)
				goto controller_busy;
			if (ret != -ENOMEM)
				return ret;

			if (rs > ODL_TB5_RING_SIZE_MIN) {
				pr_warn("odl_tb5: NHI ring setup failed at size %u; "
					"probing minimum size %u once\n", rs,
					ODL_TB5_RING_SIZE_MIN);
				rs = ODL_TB5_RING_SIZE_MIN;
				ret = odl_tb5_rings_alloc(dev, rs);
				if (!ret)
					goto alloc_dma_bufs;
				odl_tb5_rings_free(dev);
				if (ret == -EBUSY)
					goto controller_busy;
			}

			pr_err("odl_tb5: NHI ring allocation unavailable even at "
				"minimum size; a competing owner such as "
				"thunderbolt_ibverbs is one possible cause\n");
			return ret;
		}

alloc_dma_bufs:
		ret = odl_tb5_dma_bufs_alloc(dev);
		if (!ret)
			return 0;

		odl_tb5_dma_bufs_free(dev);
		odl_tb5_rings_free(dev);

		if (rs <= ODL_TB5_RING_SIZE_MIN)
			return ret;

		rs /= 2;
		pr_warn("odl_tb5: DMA buffer allocation failed (%d), retrying "
			"with odl_ring_size=%u\n", ret, rs);
	}

controller_busy:
	/* The ring allocator already unwound its partial path.  A smaller DMA
	 * batch cannot make an output HopID free. */
	pr_err("odl_tb5: controller resources unavailable; not retrying "
		"the DMA buffer size (unload a competing owner such as "
		"thunderbolt_ibverbs)\n");
	return ret;
}

static int odl_tb5_probe(struct tb_service *svc,
			 const struct tb_service_id *id)
{
	if (odl_max_devices > 0 &&
	    atomic_inc_return(&odl_bound_count) > odl_max_devices) {
		atomic_dec(&odl_bound_count);
		pr_info("odl_tb5: max_devices=%d reached, skipping service\n",
			odl_max_devices);
		return -ENODEV;
	}
	struct odl_tb5_device *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->svc = svc;
	dev->xd  = tb_service_parent(svc);

	/* Stable, per-controller numbering: /dev/odl_tb5_N == TB domain N
	 * (each controller = one NHI = one cable).  Probe order is racy
	 * across controllers, which made probe-order indexes flaky for
	 * cross-host pairing; the domain index is deterministic per NIC.
	 * Falls back to a free slot if the controller index is taken. */
	dev->index = dev->xd->tb->index;
	if (dev->index >= 0 && dev->index < ODL_TB5_MAX_DEVICES) {
		ret = ida_alloc_range(&odl_tb5_ida, dev->index, dev->index,
				      GFP_KERNEL);
		if (ret < 0)
			dev->index = -1;
		else
			dev->index = ret;
	} else {
		dev->index = -1;
	}
	if (dev->index < 0) {
		ret = ida_alloc_max(&odl_tb5_ida, ODL_TB5_MAX_DEVICES - 1,
				    GFP_KERNEL);
		if (ret < 0) {
			kfree(dev);
			return ret;
		}
		dev->index = ret;
	}

	/* Multi-path: initialise ALL slots up front (cheap, avoids special
	 * casing elsewhere).  num_paths is the configured stripe count;
	 * rings_alloc may reduce it if a higher path can't get NHI rings.
	 * negotiated/tx_active/remote start at 1 so the single-path fallback
	 * (and any code reading them before handshake) is well-defined. */
	dev->num_paths = odl_num_paths;
	dev->remote_path_count = 1;
	dev->negotiated_paths = 1;
	dev->tx_active_paths = 1;
	{
		int p;

		for (p = 0; p < ODL_TB5_MAX_PATHS; p++) {
			struct odl_tb5_path *path = &dev->paths[p];

			path->tx.dev = dev;
			path->rx.dev = dev;
			path->local_tx_hopid = -1;
			path->remote_tx_hopid = 0;
			path->stale_remote_tx_hopid = 0;
			path->in_hopid_valid = false;
			spin_lock_init(&path->tx.lock);
			spin_lock_init(&path->rx.lock);
			init_waitqueue_head(&path->tx.waitq);
			init_waitqueue_head(&path->rx.waitq);
			init_waitqueue_head(&path->rx_repost_waitq);
			atomic_set(&path->tx.completed, 0);
			atomic_set(&path->tx.submitted, 0);
			atomic_set(&path->rx.completed, 0);
			atomic_set(&path->rx.submitted, 0);
			atomic_set(&path->rx_posted, 0);
			atomic_set(&path->legacy_rx_posted, 0);
			atomic_set(&path->rx_reposting, 0);
			path->rx_repost_pending = false;
			path->rx_target = 0;
		}
	}

	dev->state = ODL_TB5_STATE_DISCONNECTED;

	mutex_init(&dev->state_lock);
	init_waitqueue_head(&dev->state_waitq);
	atomic_set(&dev->open_count, 0);
	atomic_set(&dev->pong_mask, 0);
	atomic_set(&dev->ack_mask, 0);
	atomic_set(&dev->verify_ping_mask, 0);
	atomic_set(&dev->verify_pong_mask, 0);

	/* Stream management init */
	hash_init(dev->streams);
	ida_init(&dev->stream_ida);
	mutex_init(&dev->stream_lock);
	mutex_init(&dev->dmabuf_tx_lock);
	mutex_init(&dev->dmabuf_rx_lock);
	INIT_WORK(&dev->tx_drain_work, odl_tb5_tx_drain_work_fn);
	/* rx_posted_min is a low-water mark: start high so the first real
	 * value wins. The rest are plain counters and kzalloc zeroed them.
	 * (rx_posted / rx_target themselves live per-path.) */
	atomic_set(&dev->rx_posted_min, INT_MAX);

	atomic_set(&dev->removing, 0);

	/* Adaptive TX mode defaults.
	 *
	 * Watermarks gate the shared frame pool (ODL_TB5_FRAME_POOL_SIZE
	 * slots), NOT the NHI ring depth.  With a large odl_ring_size the raw
	 * ring*3/4 exceeds the pool, so the adaptive logic can never trip and
	 * TX flow control is miscalibrated.  Clamp to the usable pool. */
	dev->tx_adaptive.mode = ODL_TB5_TX_LATENCY;
	dev->tx_adaptive.consecutive_low = 0;
	dev->tx_adaptive.high_watermark =
		min_t(unsigned int, odl_ring_size * 3 / 4,
		      ODL_TB5_FRAME_POOL_SIZE - ODL_TB5_TX_POOL_RESERVE);
	dev->tx_adaptive.low_watermark  =
		min_t(unsigned int, odl_ring_size / 4,
		      (ODL_TB5_FRAME_POOL_SIZE - ODL_TB5_TX_POOL_RESERVE) / 2);

	ret = odl_tb5_chardev_create(dev);
	if (ret) {
		pr_err("odl_tb5: chardev create failed for index %d: %d\n",
		       dev->index, ret);
		goto err_free_dev;
	}

	ret = odl_tb5_probe_alloc_rings_bufs(dev);
	if (ret) {
		pr_err("odl_tb5: ring/DMA buf alloc failed for index %d: %d\n",
		       dev->index, ret);
		goto err_chardev;
	}

	ret = odl_tb5_proto_init(dev);
	if (ret) {
		pr_err("odl_tb5: proto init failed for index %d: %d\n",
		       dev->index, ret);
		goto err_dma;
	}

	mutex_lock(&odl_tb5_devices_lock);
	list_add_tail(&dev->list, &odl_tb5_devices_list);
	mutex_unlock(&odl_tb5_devices_lock);

	tb_service_set_drvdata(svc, dev);

	pr_info("odl_tb5: probed device index %d on xdomain %pUb\n",
		dev->index, dev->xd->remote_uuid);

	return 0;

err_dma:
	odl_tb5_dma_bufs_free(dev);
	odl_tb5_rings_free(dev);
err_chardev:
	odl_tb5_chardev_destroy(dev);
err_free_dev:
	if (odl_max_devices > 0)
		atomic_dec(&odl_bound_count);
	ida_free(&odl_tb5_ida, dev->index);
	kfree(dev);
	return ret;
}

static void odl_tb5_remove(struct tb_service *svc)
{
	struct odl_tb5_device *dev = tb_service_get_drvdata(svc);
	enum odl_tb5_conn_state saved_state;

	if (!dev)
		return;

	/* Shutdown/reboot fast-path.  On reboot this runs from
	 *   device_shutdown -> pci_device_shutdown(nhi) -> tb_domain_remove ->
	 *   tb_xdomain_remove -> unregister_service -> device_release_driver
	 * i.e. the entire Thunderbolt stack is being torn down under us.  The
	 * normal teardown calls odl_tb5_rings_stop() -> tb_ring_stop(), which
	 * blocks in flush_work() waiting for ring work the already-dying NHI will
	 * never run again — wedging systemd-shutdown in D state with no console,
	 * pinging but no sshd, recoverable only by a physical power cycle (this
	 * is the ACTUAL stack we captured, not the domain-mutex path guessed at
	 * earlier).  The imminent reboot resets the NHI and all of memory, so the
	 * only safe move is to touch nothing and return: leaked structs and
	 * in-flight DMA are irrelevant when the controller is about to reset. */
	if (system_state != SYSTEM_RUNNING) {
		pr_info("odl_tb5: shutdown fast-path — skipping teardown (index %d)\n",
			dev->index);
		return;
	}

	/* Set `removing` while holding devices_lock: the protocol
	 * handler checks the flag and schedules restart/connect work
	 * strictly under this lock, so once we've cycled the lock no
	 * incoming packet can arm work on this device anymore.  Without
	 * this fence a peer reload (login/logout packet) could schedule
	 * restart_work AFTER the cancels below, and the work would then
	 * run on a torn-down/freed device — kworker crash, rmmod stuck
	 * in D-state, refcount -1, power cycle required. */
	mutex_lock(&odl_tb5_devices_lock);
	atomic_set(&dev->removing, 1);
	mutex_unlock(&odl_tb5_devices_lock);
	spin_lock_irq(&dev->verify_reply_lock);
	dev->verify_reply_open = false;
	dev->drain_reply_open = false;
	spin_unlock_irq(&dev->verify_reply_lock);
	wake_up_all(&dev->verify_waitq);

	mutex_lock(&dev->state_lock);
	saved_state = dev->state;
	dev->state = ODL_TB5_STATE_DISCONNECTED;
	wake_up_all(&dev->state_waitq);
	mutex_unlock(&dev->state_lock);

	if (saved_state == ODL_TB5_STATE_CONNECTED ||
	    saved_state == ODL_TB5_STATE_READY)
		odl_tb5_proto_send_logout(dev);

	odl_tb5_poll_disarm(dev);

	/* Two cancel passes: the works arm each other (restart→login,
	 * login→connect, connect→login/verify).  A work that was already
	 * running before `removing` was set may re-arm a work we canceled
	 * earlier in the same pass; because every work fn gates its
	 * scheduling on !removing, anything re-armed during pass 1 runs
	 * as a no-op — pass 2 only makes sure nothing is left PENDING
	 * when the device is freed. */
	cancel_work_sync(&dev->verify_work);
	cancel_work_sync(&dev->ctrl_reply_work);
	cancel_work_sync(&dev->restart_work);
	cancel_work_sync(&dev->connect_work);
	cancel_delayed_work_sync(&dev->login_work);
	cancel_work_sync(&dev->tx_drain_work);

	cancel_work_sync(&dev->verify_work);
	cancel_work_sync(&dev->ctrl_reply_work);
	cancel_work_sync(&dev->restart_work);
	cancel_work_sync(&dev->connect_work);
	cancel_delayed_work_sync(&dev->login_work);
	cancel_work_sync(&dev->tx_drain_work);

	odl_tb5_rings_stop(dev);

	/* Disable the DMA paths while rings and hopids are still valid —
	 * rings_free() below NULLs the rings and releases local_tx_hopid,
	 * which would make this call a no-op and leave stale paths in the
	 * routers (breaks the next module load until a controller reset).
	 * in_hopid_valid marks a fully-enabled path; iterate every one.
	 *
	 * Release regardless of connection state (upstream BUG1): gating this
	 * on CONNECTED/READY meant removal during HANDSHAKE (login retrying,
	 * peer gone, admin unbind) leaked the hop-ID until enable_paths
	 * returned -ENOMEM on every later load.  in_hopid_valid is already the
	 * authoritative record of what was actually allocated, so the state
	 * check adds nothing but the leak. */
	{
		int p;

		for (p = 0; p < dev->num_paths; p++) {
			struct odl_tb5_path *path = &dev->paths[p];

			if (!path->in_hopid_valid)
				continue;

			tb_xdomain_disable_paths(dev->xd,
						 path->local_tx_hopid,
						 path->tx.ring ? path->tx.ring->hop : -1,
						 path->remote_tx_hopid,
						 path->rx.ring ? path->rx.ring->hop : -1);
			/* restart_work may have released the in-hopid already
			 * (a restart can race us) — releasing twice trips
			 * ida_free's WARN. */
			tb_xdomain_release_in_hopid(dev->xd,
						    path->remote_tx_hopid);
			path->in_hopid_valid = false;
		}
	}

	synchronize_rcu();

	mutex_lock(&odl_tb5_devices_lock);
	list_del_rcu(&dev->list);
	mutex_unlock(&odl_tb5_devices_lock);

	odl_tb5_streams_destroy_all(dev);
	ida_destroy(&dev->stream_ida);

	odl_tb5_frame_pool_free(dev);
	odl_tb5_batch_pool_free(dev);
	odl_tb5_dma_bufs_free(dev);
	odl_tb5_rings_free(dev);

	odl_tb5_chardev_destroy(dev);

	/*
	 * Give the slot back. odl_tb5_probe() takes one from odl_bound_count
	 * and the probe error path returns it, but this — the normal removal
	 * path — did not. With max_devices=1 that leaks the only slot the
	 * moment the peer goes away, so every later probe is rejected with
	 * "max_devices reached, skipping service" and the link can never come
	 * back on its own.
	 *
	 * Observed exactly that: peer restarted, this node logged "removed
	 * device index 0", then refused the replacement service two seconds
	 * later and sat printing "incoming packet route 2 — no matching
	 * device" indefinitely. The only escape was reloading the module,
	 * which on this hardware is the risky operation this leak forces you
	 * into.
	 */
	if (odl_max_devices > 0)
		atomic_dec(&odl_bound_count);

	pr_info("odl_tb5: removed device index %d\n", dev->index);

	ida_free(&odl_tb5_ida, dev->index);
	kfree(dev);
}

static struct tb_service_driver odl_tb5_driver = {
	.driver.name	= "odl_tb5",
	.probe		= odl_tb5_probe,
	.remove		= odl_tb5_remove,
	.id_table	= odl_tb5_ids,
};

static int __init odl_tb5_init(void)
{
	int ret;

	if (!is_power_of_2(odl_ring_size) ||
	    odl_ring_size < ODL_TB5_RING_SIZE_MIN ||
	    odl_ring_size > ODL_TB5_RING_SIZE_MAX) {
		pr_err("odl_tb5: invalid ring_size=%u (must be power-of-2, %u-%u)\n",
		       odl_ring_size, ODL_TB5_RING_SIZE_MIN,
		       ODL_TB5_RING_SIZE_MAX);
		return -EINVAL;
	}

	if (odl_num_paths < 1 || odl_num_paths > ODL_TB5_MAX_PATHS) {
		unsigned int clamped = clamp_t(unsigned int, odl_num_paths,
					       1, ODL_TB5_MAX_PATHS);
		pr_warn("odl_tb5: num_paths=%u out of range (1..%u), clamping to %u\n",
			odl_num_paths, ODL_TB5_MAX_PATHS, clamped);
		odl_num_paths = clamped;
	}

	ret = odl_tb5_chardev_init();
	if (ret)
		return ret;

	/* Create the debugfs root before any device probe so per-device
	 * subdirs have a parent. Non-fatal if debugfs is unavailable. */
	odl_tb5_debugfs_root = debugfs_create_dir("odl_tb5", NULL);

	/* If loopback=1 or more, create software-only devices.
	 * Loopback devices work without Thunderbolt hardware and
	 * don't need property directories or service registration. */
	if (odl_loopback_count > 0) {
		ret = odl_loopback_init();
		if (ret)
			goto err_chardev;
		pr_info("odl_tb5: loopback mode enabled (%d devices)\n",
			odl_loopback_count);
		return 0;
	}

	odl_tb5_property_dir = tb_property_create_dir(&odl_tb5_proto_uuid);
	if (!odl_tb5_property_dir) {
		ret = -ENOMEM;
		goto err_chardev;
	}

	/* Choose protocol ID based on mode */
	u32 protocol_id = ODL_TB5_PROTOCOL_ID;
	if (odl_protocol_mode == 1)
		protocol_id = ODL_TB5_PROTOCOL_ID_APPLE;

	const char *protocol_key = ODL_TB5_PROTOCOL_KEY;
	if (odl_protocol_mode == 1)
		protocol_key = ODL_TB5_PROTOCOL_KEY_APPLE;

	ret = tb_property_add_immediate(odl_tb5_property_dir, "prtcid",
					protocol_id);
	if (ret)
		goto err_dir;

	ret = tb_property_add_immediate(odl_tb5_property_dir, "prtcvers",
					ODL_TB5_PROTOCOL_VER);
	if (ret)
		goto err_dir;

	ret = tb_property_add_immediate(odl_tb5_property_dir, "prtcrevs", 1);
	if (ret)
		goto err_dir;

	ret = tb_property_add_immediate(odl_tb5_property_dir, "prtcstns", 0);
	if (ret)
		goto err_dir;

	ret = tb_register_property_dir(protocol_key,
				       odl_tb5_property_dir);
	if (ret)
		goto err_dir;

	/* In Apple mode, also register under OdinLink's original key so we
	 * can still talk to other OdinLink nodes. Need a separate directory
	 * since the same dir can't be registered under two keys. */
	if (odl_protocol_mode == 1) {
		odl_tb5_apple_property_dir =
			tb_property_create_dir(&odl_tb5_proto_uuid);
		if (!odl_tb5_apple_property_dir) {
			ret = -ENOMEM;
			goto err_dir;
		}
		tb_property_add_immediate(odl_tb5_apple_property_dir,
					  "prtcid", ODL_TB5_PROTOCOL_ID);
		tb_property_add_immediate(odl_tb5_apple_property_dir,
					  "prtcvers", ODL_TB5_PROTOCOL_VER);
		tb_property_add_immediate(odl_tb5_apple_property_dir,
					  "prtcrevs", 1);
		tb_property_add_immediate(odl_tb5_apple_property_dir,
					  "prtcstns", 0);
		ret = tb_register_property_dir(ODL_TB5_PROTOCOL_KEY,
					odl_tb5_apple_property_dir);
		if (ret)
			goto err_dir;
	}

	odl_tb5_proto_register();

	ret = tb_register_service_driver(&odl_tb5_driver);
	if (ret)
		goto err_proto;

	pr_info("odl_tb5: service registered (ring_size=%u); awaiting peer probe\n",
		odl_ring_size);

	return 0;

err_proto:
	odl_tb5_proto_unregister();
err_dir:
	if (odl_tb5_apple_property_dir) {
		tb_property_free_dir(odl_tb5_apple_property_dir);
		odl_tb5_apple_property_dir = NULL;
	}
	tb_property_free_dir(odl_tb5_property_dir);
err_chardev:
	debugfs_remove_recursive(odl_tb5_debugfs_root);
	odl_tb5_debugfs_root = NULL;
	odl_tb5_chardev_exit();
	return ret;
}

static void __exit odl_tb5_exit(void)
{
	struct odl_tb5_device *dev, *tmp;

	/* Clean up software loopback devices first (no NHI/hardware deps) */
	if (odl_loopback_count > 0) {
		odl_loopback_exit();
		goto out;
	}

	/* Unregister first so tb core removes bound services before orphan cleanup. */
	tb_unregister_service_driver(&odl_tb5_driver);

	mutex_lock(&odl_tb5_devices_lock);
	list_for_each_entry_safe(dev, tmp, &odl_tb5_devices_list, list) {
		pr_warn("odl_tb5: cleaning up orphaned device at exit\n");
		list_del_rcu(&dev->list);
		atomic_set(&dev->removing, 1);
		spin_lock_irq(&dev->verify_reply_lock);
		dev->verify_reply_open = false;
		dev->drain_reply_open = false;
		spin_unlock_irq(&dev->verify_reply_lock);
		wake_up_all(&dev->verify_waitq);
		odl_tb5_poll_disarm(dev);
		/* Double cancel pass — see odl_tb5_remove() for why. */
		cancel_work_sync(&dev->verify_work);
		cancel_work_sync(&dev->ctrl_reply_work);
		cancel_work_sync(&dev->restart_work);
		cancel_work_sync(&dev->connect_work);
		cancel_delayed_work_sync(&dev->login_work);
		cancel_work_sync(&dev->tx_drain_work);
		cancel_work_sync(&dev->verify_work);
		cancel_work_sync(&dev->ctrl_reply_work);
		cancel_work_sync(&dev->restart_work);
		cancel_work_sync(&dev->connect_work);
		cancel_delayed_work_sync(&dev->login_work);
		cancel_work_sync(&dev->tx_drain_work);
		odl_tb5_rings_stop(dev);
		synchronize_rcu();
		odl_tb5_streams_destroy_all(dev);
		ida_destroy(&dev->stream_ida);
		odl_tb5_frame_pool_free(dev);
		odl_tb5_batch_pool_free(dev);
		odl_tb5_dma_bufs_free(dev);
		odl_tb5_rings_free(dev);
		odl_tb5_chardev_destroy(dev);
		ida_free(&odl_tb5_ida, dev->index);
		kfree(dev);
	}
	mutex_unlock(&odl_tb5_devices_lock);

	odl_tb5_proto_unregister();

	/* Unregister property dirs: main dir under its protocol key,
	 * and the Apple backup dir under OdinLink key if dual-mode */
	const char *main_key = (odl_protocol_mode == 1)
		? ODL_TB5_PROTOCOL_KEY_APPLE : ODL_TB5_PROTOCOL_KEY;

	tb_unregister_property_dir(main_key, odl_tb5_property_dir);
	tb_property_free_dir(odl_tb5_property_dir);

	if (odl_tb5_apple_property_dir) {
		tb_unregister_property_dir(ODL_TB5_PROTOCOL_KEY,
					   odl_tb5_apple_property_dir);
		tb_property_free_dir(odl_tb5_apple_property_dir);
	}
out:
	/*
	 * Streams are freed with kfree_rcu(), so a grace period may still be
	 * outstanding here. kfree_rcu() is serviced by the core kernel rather
	 * than by module text, so unloading cannot jump into freed code — but
	 * waiting is cheap at unload and leaves nothing in flight against a
	 * module that is going away.
	 */
	rcu_barrier();

	debugfs_remove_recursive(odl_tb5_debugfs_root);
	odl_tb5_debugfs_root = NULL;
	odl_tb5_chardev_exit();
	ida_destroy(&odl_tb5_ida);
	pr_info("odl_tb5: OdinLink TB5 driver unloaded\n");
}

module_init(odl_tb5_init);
module_exit(odl_tb5_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("OdinLink Team");
MODULE_DESCRIPTION("OdinLink Thunderbolt 5 DMA Ring Driver");
MODULE_IMPORT_NS("DMA_BUF");
