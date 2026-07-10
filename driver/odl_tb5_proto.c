// SPDX-License-Identifier: MIT
/*
 * OdinLink — The Handshake: How Two Machines Agree to Talk
 *
 * Before any data moves, both sides need to agree on which DMA slots to
 * use. This is the XDomain login/logout protocol:
 *
 *   1. Each machine advertises its protocol ID ("OdinLink" = 0x4F4C, or
 *      "Apple" = 0xFA57 for macOS compat) so the other end can find it.
 *   2. Both sides send a "login" request saying "hey, use DMA slot X for
 *      your outgoing data" and wait for the other to reply.
 *   3. When both have sent and received a login, we enable the paths
 *      (like opening a gate), start the DMA rings, and sanity-check the
 *      link with a ping/pong exchange.
 *   4. If either side disconnects, a "logout" tears it all down so the
 *      next connection starts fresh.
 *
 * This is the same pattern Linux uses for Thunderbolt networking —
 * just with our own protocol ID so we don't collide with IP-over-TB.
 */

#include "odl_tb5_core.h"
#include <linux/delay.h>

#define ODL_TB5_MSG_LOGIN      1
#define ODL_TB5_MSG_LOGIN_RSP  2
#define ODL_TB5_MSG_LOGOUT     3

#define ODL_TB5_LOGIN_TIMEOUT  500
#define ODL_TB5_ENABLE_RETRIES 5
#define ODL_TB5_ENABLE_DELAY   200

struct odl_tb5_xd_header {
	u32	route_hi;
	u32	route_lo;
	u32	length_sn;
	uuid_t	uuid;
	u32	type;
};

/*
 * Login v2 (multi-path striping): the v1 layout is kept bit-for-bit and
 * the multi-path fields are APPENDED.  Detection is purely by SIZE — the
 * protocol version stays at 1, so old and new drivers interoperate
 * without a flag day.  Path 0's hopid stays in transmit_path; paths
 * 1..N-1 go into extra_tx_hopids[].
 */
struct odl_tb5_login_msg {
	struct odl_tb5_xd_header xd_hdr;
	u32 proto_version;
	u32 transmit_path;
	u32 reserved[2];
	/* v2 extension (presence detected via packet size) */
	u32 path_count;
	u32 extra_tx_hopids[ODL_TB5_MAX_PATHS - 1];
};

struct odl_tb5_login_response {
	struct odl_tb5_xd_header xd_hdr;
	u32 status;
	u32 transmit_path;
	u32 reserved[2];
	/* v2 extension (presence detected via the XD header length field) */
	u32 path_count;
	u32 extra_tx_hopids[ODL_TB5_MAX_PATHS - 1];
};

/* Size of the v1 (single-path) login message — everything before the v2
 * extension.  Old peers send exactly this many bytes. */
#define ODL_TB5_LOGIN_V1_SIZE	offsetof(struct odl_tb5_login_msg, path_count)

struct odl_tb5_logout_msg {
	struct odl_tb5_xd_header xd_hdr;
};

static void odl_tb5_login_work_fn(struct work_struct *work);
static void odl_tb5_connect_work_fn(struct work_struct *work);
static void odl_tb5_restart_work_fn(struct work_struct *work);
static int  odl_tb5_complete_connection(struct odl_tb5_device *dev);

#define XD_HDR_SIZE_DW  3
#define XD_SN_MASK      0x18000000u
/* Payload length (in dwords) lives in the low 6 bits of length_sn — used
 * to detect whether a login response carries the v2 multi-path fields
 * (the ctl layer copies a fixed-size buffer, so the received byte count
 * is not otherwise visible to us). */
#define XD_LEN_MASK     0x3fu

static void odl_tb5_xd_header_init(struct odl_tb5_xd_header *hdr,
				    struct tb_xdomain *xd, u32 type,
				    size_t total_size)
{
	hdr->route_hi  = upper_32_bits(xd->route);
	hdr->route_lo  = lower_32_bits(xd->route);
	hdr->length_sn = total_size / 4 - XD_HDR_SIZE_DW;
	hdr->uuid      = odl_tb5_proto_uuid;
	hdr->type      = type;
}

static struct odl_tb5_device *odl_tb5_find_device_by_route(u64 route)
{
	struct odl_tb5_device *dev;

	list_for_each_entry(dev, &odl_tb5_devices_list, list) {
		if (dev->xd->route == route)
			return dev;
	}
	return NULL;
}

static int odl_tb5_proto_handle_packet(const void *buf, size_t size,
				       void *data)
{
	const struct odl_tb5_xd_header *hdr = buf;
	struct odl_tb5_device *dev;
	u64 route;
	bool need_complete = false;

	if (size < sizeof(*hdr))
		return 0;

	route = (((u64)hdr->route_hi << 32) | hdr->route_lo) & ~BIT_ULL(63);

	mutex_lock(&odl_tb5_devices_lock);
	dev = odl_tb5_find_device_by_route(route);

	if (!dev) {
		mutex_unlock(&odl_tb5_devices_lock);
		pr_warn("OdinLink: incoming packet route %llx — no matching device\n",
			route);
		return 0;
	}

	/* Device is being torn down: don't touch its state and, above
	 * all, don't schedule any work on it.  remove() sets `removing`
	 * under devices_lock, so this check (plus scheduling only while
	 * holding the lock below) guarantees no work is armed after
	 * remove()'s cancel_work_sync() calls have run. */
	if (atomic_read(&dev->removing)) {
		mutex_unlock(&odl_tb5_devices_lock);
		return 1;
	}

	switch (hdr->type) {
	case ODL_TB5_MSG_LOGIN: {
		const struct odl_tb5_login_msg *pkg = buf;
		struct odl_tb5_login_response resp = { };
		u32 remote_hopids[ODL_TB5_MAX_PATHS] = { };
		int remote_paths = 1;
		u32 remote_tx_hopid = 0;
		u32 proto_ver = 0;
		int ret, p;

		/* Minimum: XDomain header (40 bytes). The payload fields
		 * (transmit_path, proto_version) may be absent if the peer
		 * uses a different login format (e.g., Apple ThunderboltRDMA).
		 * Default missing fields to 0 and proceed. */
		if (size < sizeof(struct odl_tb5_xd_header)) {
			mutex_unlock(&odl_tb5_devices_lock);
			return 0;
		}

		if (size >= sizeof(*pkg)) {
			/* Login v2: multi-path fields present (by SIZE,
			 * proto_version stays 1 — no flag day). */
			remote_tx_hopid = pkg->transmit_path;
			proto_ver = pkg->proto_version;
			remote_paths = clamp_t(int, (int)pkg->path_count,
					       1, ODL_TB5_MAX_PATHS);
			for (p = 1; p < remote_paths; p++)
				remote_hopids[p] = pkg->extra_tx_hopids[p - 1];
		} else if (size >= ODL_TB5_LOGIN_V1_SIZE) {
			/* Login v1: single path */
			remote_tx_hopid = pkg->transmit_path;
			proto_ver = pkg->proto_version;
		} else if (size >= sizeof(struct odl_tb5_xd_header) + 8) {
			/* Apple-style: transmit_path at offset 40, version at 44 */
			const u32 *payload = (const u32 *)(hdr + 1);
			remote_tx_hopid = payload[1];
			proto_ver = payload[0];
		}
		remote_hopids[0] = remote_tx_hopid;

		pr_info("OdinLink: received login from peer "
			"(version=%u, tx_path=%u, size=%zu)\n",
			proto_ver, remote_tx_hopid, size);
		if (remote_paths > 1)
			pr_info("OdinLink: peer advertises %d paths\n",
				remote_paths);

		resp.xd_hdr.route_hi  = upper_32_bits(dev->xd->route);
		resp.xd_hdr.route_lo  = lower_32_bits(dev->xd->route);
		resp.xd_hdr.length_sn =
			(hdr->length_sn & XD_SN_MASK) |
			(sizeof(resp) / 4 - XD_HDR_SIZE_DW);
		resp.xd_hdr.uuid      = odl_tb5_proto_uuid;
		resp.xd_hdr.type      = ODL_TB5_MSG_LOGIN_RSP;
		resp.status            = 0;
		resp.transmit_path     = dev->paths[0].local_tx_hopid;
		/* Always advertise all of our own paths; the peer clamps
		 * to its own count.  Old peers simply ignore the extra
		 * bytes (their response buffer is the short v1 struct). */
		resp.path_count        = dev->num_paths;
		for (p = 1; p < dev->num_paths; p++)
			resp.extra_tx_hopids[p - 1] =
				dev->paths[p].local_tx_hopid;

		ret = tb_xdomain_response(dev->xd, &resp, sizeof(resp),
					  TB_CFG_PKG_XDOMAIN_RESP);
		pr_info("OdinLink: sent login response (ret=%d, route=%llx, "
			"sn=%u, tx_hopid=%d)\n",
			ret, dev->xd->route,
			(hdr->length_sn & XD_SN_MASK) >> 27,
			dev->paths[0].local_tx_hopid);

		mutex_lock(&dev->state_lock);
		if (dev->state == ODL_TB5_STATE_READY) {
			/* We were fully up and the peer is logging in again —
			 * a genuine peer restart.  Tear down and re-handshake. */
			pr_info("OdinLink: peer restarted (our state=%d), "
				"scheduling restart\n", dev->state);
			for (p = 0; p < ODL_TB5_MAX_PATHS; p++)
				dev->paths[p].stale_remote_tx_hopid =
					dev->paths[p].remote_tx_hopid;
			for (p = 0; p < remote_paths; p++)
				dev->paths[p].remote_tx_hopid =
					remote_hopids[p];
			dev->remote_path_count = remote_paths;
			dev->negotiated_paths = min(dev->num_paths,
						    remote_paths);
			dev->login_received = true;
			mutex_unlock(&dev->state_lock);
			/* Schedule while still holding devices_lock so
			 * remove() can fence us out (see removing check
			 * above). */
			schedule_work(&dev->restart_work);
			mutex_unlock(&odl_tb5_devices_lock);
			return 1;
		}

		/* state HANDSHAKE or CONNECTED: the peer is (still) handshaking.
		 * Record its hopids and login, but do NOT restart on a login
		 * received while CONNECTED — that just means the peer is catching
		 * up during our verify window.  Restarting here livelocks: each
		 * side keeps kicking the other out of verify.  We already sent
		 * our login response above; connect_work is idempotent. */
		for (p = 0; p < remote_paths; p++)
			dev->paths[p].remote_tx_hopid = remote_hopids[p];
		dev->remote_path_count = remote_paths;
		dev->negotiated_paths = min(dev->num_paths, remote_paths);
		dev->login_received = true;
		if (dev->login_sent && dev->state == ODL_TB5_STATE_HANDSHAKE)
			need_complete = true;
		mutex_unlock(&dev->state_lock);

		if (need_complete)
			schedule_work(&dev->connect_work);
		mutex_unlock(&odl_tb5_devices_lock);

		return 1;
	}

	case ODL_TB5_MSG_LOGOUT: {
		int p;

		pr_info("OdinLink: received logout from peer\n");

		mutex_lock(&dev->state_lock);
		for (p = 0; p < ODL_TB5_MAX_PATHS; p++)
			dev->paths[p].stale_remote_tx_hopid =
				dev->paths[p].remote_tx_hopid;
		dev->login_received = false;
		dev->login_sent = false;
		mutex_unlock(&dev->state_lock);

		/* Schedule under devices_lock — see removing check above. */
		schedule_work(&dev->restart_work);
		mutex_unlock(&odl_tb5_devices_lock);

		return 1;
	}

	default:
		mutex_unlock(&odl_tb5_devices_lock);
		return 0;
	}
}

static struct tb_protocol_handler odl_tb5_handler = {
	.uuid     = &odl_tb5_proto_uuid,
	.callback = odl_tb5_proto_handle_packet,
};

int odl_tb5_proto_register(void)
{
	return tb_register_protocol_handler(&odl_tb5_handler);
}

void odl_tb5_proto_unregister(void)
{
	tb_unregister_protocol_handler(&odl_tb5_handler);
}

/* Send a login request to the peer and process the response. */
int odl_tb5_proto_send_login(struct odl_tb5_device *dev)
{
	struct odl_tb5_login_msg msg = { };
	struct odl_tb5_login_response resp = { };
	bool need_complete = false;
	bool still_waiting = false;
	int remote_paths = 1;
	u32 resp_dw;
	int ret, p;

	odl_tb5_xd_header_init(&msg.xd_hdr, dev->xd, ODL_TB5_MSG_LOGIN,
			       sizeof(msg));
	msg.proto_version = ODL_TB5_PROTOCOL_VER;
	msg.transmit_path = dev->paths[0].local_tx_hopid;
	msg.path_count = dev->num_paths;
	for (p = 1; p < dev->num_paths; p++)
		msg.extra_tx_hopids[p - 1] = dev->paths[p].local_tx_hopid;

	ret = tb_xdomain_request(dev->xd, &msg, sizeof(msg),
				 TB_CFG_PKG_XDOMAIN_REQ,
				 &resp, sizeof(resp),
				 TB_CFG_PKG_XDOMAIN_RESP,
				 ODL_TB5_LOGIN_TIMEOUT);
	if (ret) {
		pr_warn("OdinLink: login request failed: %d\n", ret);
		return ret;
	}

	/* In Apple protocol mode, be lenient with response format.
	 * The peer might use a different UUID and response structure.
	 * Accept any response that follows the XDomain response format. */
	if (odl_protocol_mode == 1) {
		/* Apple mode: accept any response that looks reasonable.
		 * The transmit_path is at a fixed offset in the response.
		 * Always single-path. */
		mutex_lock(&dev->state_lock);
		dev->paths[0].remote_tx_hopid = resp.transmit_path;
		goto login_ok;
	}

	if (!uuid_equal(&resp.xd_hdr.uuid, &odl_tb5_proto_uuid)) {
		pr_warn("OdinLink: login response UUID mismatch\n");
		return -EPROTO;
	}

	if (resp.xd_hdr.type != ODL_TB5_MSG_LOGIN_RSP) {
		pr_warn("OdinLink: unexpected response type %u\n",
			resp.xd_hdr.type);
		return -EPROTO;
	}

	if (resp.status != 0) {
		pr_warn("OdinLink: peer rejected login with status %u\n",
			resp.status);
		return -ECONNREFUSED;
	}

	/* The unlock at login_ok below pairs with this (and the Apple-mode
	 * lock above) — historically this function unlocked state_lock
	 * without ever taking it. */
	mutex_lock(&dev->state_lock);
	dev->paths[0].remote_tx_hopid = resp.transmit_path;

	/* Multi-path (v2) detection is size-tolerant: the responder encodes
	 * its payload length in the XD header.  Old responders send the
	 * short v1 layout — the extension bytes in `resp` are then
	 * unspecified (the ctl layer copies a fixed-size buffer), so never
	 * read them unless the advertised length covers them. */
	resp_dw = resp.xd_hdr.length_sn & XD_LEN_MASK;
	if (resp_dw >= sizeof(resp) / 4 - XD_HDR_SIZE_DW) {
		remote_paths = clamp_t(int, (int)resp.path_count,
				       1, ODL_TB5_MAX_PATHS);
		for (p = 1; p < remote_paths; p++)
			dev->paths[p].remote_tx_hopid =
				resp.extra_tx_hopids[p - 1];
		if (remote_paths > 1)
			pr_info("OdinLink: peer response advertises %d paths\n",
				remote_paths);
	}

login_ok:
	dev->remote_path_count = remote_paths;
	dev->negotiated_paths = min(dev->num_paths, remote_paths);
	dev->login_sent = true;
	if (dev->login_received && dev->state == ODL_TB5_STATE_HANDSHAKE)
		need_complete = true;
	/* Our login succeeded but the peer's login hasn't arrived yet: keep
	 * retrying at the normal backoff so a peer that came up first (and
	 * whose own login_work already stopped) still gets poked.  Bounded,
	 * and it stops the moment we leave HANDSHAKE — unlike an unconditional
	 * fast re-login, which livelocks by restarting a mid-verify peer. */
	still_waiting = !need_complete &&
			dev->state == ODL_TB5_STATE_HANDSHAKE;
	mutex_unlock(&dev->state_lock);

	pr_info("OdinLink: login sent OK, remote_tx_hopid=%d\n",
		dev->paths[0].remote_tx_hopid);

	if (need_complete && !atomic_read(&dev->removing))
		schedule_work(&dev->connect_work);

	if (still_waiting && !atomic_read(&dev->removing))
		schedule_delayed_work(&dev->login_work,
				      msecs_to_jiffies(ODL_TB5_LOGIN_TIMEOUT));

	return 0;
}

/* Bring up one DMA path: allocate the input HopID for the peer's TX,
 * start the ring pair, and enable the router paths (with retries).
 * Returns 0 on success; on failure the path is fully rolled back. */
static int odl_tb5_connect_path(struct odl_tb5_device *dev, int idx)
{
	struct odl_tb5_path *path = &dev->paths[idx];
	int ret, i;

	ret = tb_xdomain_alloc_in_hopid(dev->xd, path->remote_tx_hopid);
	if (ret < 0) {
		pr_err("OdinLink: failed to allocate input HopID: %d\n", ret);
		return ret;
	}
	path->in_hopid_valid = true;
	ret = odl_tb5_rings_start(dev, idx);
	if (ret) {
		pr_err("OdinLink: failed to start rings: %d\n", ret);
		tb_xdomain_release_in_hopid(dev->xd, path->remote_tx_hopid);
		path->in_hopid_valid = false;
		return ret;
	}

	if (idx == 0) {
		/* Legacy 16-frame RX prime for path 0 — unchanged from the
		 * single-path flow (the legacy double-buffers only exist
		 * for path 0).  Paths > 0 get their RX window from the
		 * frame-pool repost in verify_work before any pings fly;
		 * frames the peer sends earlier are absorbed by its
		 * ping/pong retry loop. */
		size_t rx_prime = (size_t)ODL_TB5_FRAME_SIZE * 16;

		ret = odl_tb5_submit_rx(dev, 0, rx_prime);
		if (ret) {
			pr_err("OdinLink: failed to prime RX: %d\n", ret);
			odl_tb5_rings_stop_path(dev, idx);
			tb_xdomain_release_in_hopid(dev->xd,
						    path->remote_tx_hopid);
			path->in_hopid_valid = false;
			return ret;
		}
		pr_info("OdinLink: RX primed with 16 frames before "
			"enable_paths\n");
	}

	for (i = 0; i < ODL_TB5_ENABLE_RETRIES; i++) {
		ret = tb_xdomain_enable_paths(dev->xd,
					      path->local_tx_hopid,
					      path->tx.ring->hop,
					      path->remote_tx_hopid,
					      path->rx.ring->hop);
		if (!ret)
			break;

		if (i < ODL_TB5_ENABLE_RETRIES - 1) {
			pr_warn("OdinLink: enable_paths failed (%d), "
				"retry %d/%d in %d ms\n",
				ret, i + 1, ODL_TB5_ENABLE_RETRIES,
				ODL_TB5_ENABLE_DELAY);
			msleep(ODL_TB5_ENABLE_DELAY);
		}
	}

	if (ret) {
		pr_err("OdinLink: failed to enable XDomain paths "
		       "after %d attempts: %d\n",
		       ODL_TB5_ENABLE_RETRIES, ret);
		odl_tb5_rings_stop_path(dev, idx);
		tb_xdomain_release_in_hopid(dev->xd, path->remote_tx_hopid);
		path->in_hopid_valid = false;
		return ret;
	}

	return 0;
}

/* Finish handshake and bring link up. */
static int odl_tb5_complete_connection(struct odl_tb5_device *dev)
{
	int negotiated = dev->negotiated_paths;
	int enabled = 0;
	int ret, p;

	if (negotiated < 1)
		negotiated = 1;

	dev->tx_active_paths = 1;

	for (p = 0; p < negotiated; p++) {
		ret = odl_tb5_connect_path(dev, p);
		if (ret) {
			if (p == 0)
				return ret;
			/* Path i > 0 failed: don't activate this or any
			 * later path — degrade instead of hard-failing. */
			pr_warn("OdinLink: path %d bring-up failed (%d), "
				"continuing with %d path(s)\n", p, ret, p);
			break;
		}
		enabled++;
	}

	if (enabled < negotiated) {
		negotiated = enabled;
		dev->negotiated_paths = enabled;
	}

	mutex_lock(&dev->state_lock);
	dev->state = ODL_TB5_STATE_CONNECTED;
	wake_up_all(&dev->state_waitq);
	mutex_unlock(&dev->state_lock);

	pr_info("OdinLink: connected to peer "
		"(local_tx_hopid=%d, remote_tx_hopid=%d, "
		"tx_ring_hop=%d, rx_ring_hop=%d, "
		"ring_size=%d, E2E enabled)\n",
		dev->paths[0].local_tx_hopid, dev->paths[0].remote_tx_hopid,
		dev->paths[0].tx.ring->hop, dev->paths[0].rx.ring->hop,
		dev->paths[0].tx.ring_size);
	if (negotiated > 1)
		pr_info("OdinLink: %d DMA paths enabled\n", negotiated);

	/* Allocate frame pool early so verify uses the non-blocking
	 * pool path for PING/PONG instead of the legacy submit_tx
	 * which blocks waiting for TX completion (unreliable on
	 * Barlow Ridge).  With more than 2 paths the shared pool is
	 * doubled so each path still gets a useful RX window. */
	if (!dev->frame_pool.slots) {
		int pool_size = negotiated > 2 ? 2 * ODL_TB5_FRAME_POOL_SIZE
					       : ODL_TB5_FRAME_POOL_SIZE;
		int pool_ret = odl_tb5_frame_pool_alloc(dev, pool_size);

		if (pool_ret)
			pr_warn("OdinLink: frame pool alloc failed (%d), "
				"verify will use legacy path\n", pool_ret);
	}

	/* Allocate batch buffer pool for throughput-mode TX. */
	if (!dev->batch_pool.bufs[0].virt) {
		int bret = odl_tb5_batch_pool_alloc(dev);

		if (bret)
			pr_warn("OdinLink: batch pool alloc failed (%d), "
				"throughput mode disabled\n", bret);
	}

	odl_tb5_poll_kick(dev);
	if (!atomic_read(&dev->removing))
		schedule_work(&dev->verify_work);

	return 0;
}

/* Deferred connection completion in safe work context. */
static void odl_tb5_connect_work_fn(struct work_struct *work)
{
	struct odl_tb5_device *dev =
		container_of(work, struct odl_tb5_device, connect_work);
	int ret;

	if (atomic_read(&dev->removing))
		return;

	ret = odl_tb5_complete_connection(dev);
	if (ret) {
		mutex_lock(&dev->state_lock);
		dev->login_sent = false;
		dev->login_received = false;
		mutex_unlock(&dev->state_lock);

		pr_warn("OdinLink: connection completion failed (%d), "
			"retrying handshake\n", ret);
		if (!atomic_read(&dev->removing))
			schedule_delayed_work(&dev->login_work,
					      msecs_to_jiffies(1000));
	}
}

/* Write a kernel DMA control message (ping or pong) via frame pool.
 * @path_idx selects the DMA path whose TX ring carries the message. */
static int odl_tb5_send_dma_msg(struct odl_tb5_device *dev, u32 type,
				int path_idx)
{
	struct odl_tb5_frame_slot *slot;
	struct odl_tb5_dma_hdr *dhdr;
	int ret;

	/* Use frame pool for non-blocking send (each msg gets its own
	 * slot, no drain wait).  Write raw DMA header at offset 0
	 * (no stream header) — the RX side uses legacy frames during
	 * verify and expects DMA magic at offset 0. */
	if (dev->frame_pool.slots) {
		slot = odl_tb5_frame_pool_get(&dev->frame_pool);
		if (!slot)
			return -ENOMEM;

		dhdr = slot->virt;
		memset(dhdr, 0, sizeof(*dhdr));
		dhdr->magic = cpu_to_le32(ODL_TB5_DMA_MAGIC);
		dhdr->type  = cpu_to_le32(type);

		slot->frame.size = sizeof(*dhdr);
		slot->frame.sof = ODL_TB5_PDF_SOF_CTRL;
		slot->frame.eof = ODL_TB5_PDF_EOF_CTRL;
		slot->frame.callback = odl_tb5_tx_callback;
		slot->tx_msg = NULL;

		ret = tb_ring_tx(dev->paths[path_idx].tx.ring, &slot->frame);
		if (ret < 0) {
			odl_tb5_frame_pool_put(&dev->frame_pool, slot);
			return ret;
		}
		odl_tb5_tx_submitted(dev);

		atomic64_inc(&dev->stats.path_tx_frames[path_idx]);
		return 0;
	}

	/* Legacy fallback (before frame pool is allocated) — path 0 only,
	 * the legacy double-buffers don't exist for other paths. */
	{
		struct odl_tb5_dma_hdr *hdr;

		if (path_idx != 0)
			return -ENXIO;

		hdr = dev->paths[0].tx.bufs[dev->paths[0].tx.front].virt;
		memset(hdr, 0, sizeof(*hdr));
		hdr->magic = cpu_to_le32(ODL_TB5_DMA_MAGIC);
		hdr->type  = cpu_to_le32(type);

		return odl_tb5_submit_tx(dev, 0, sizeof(*hdr), true);
	}
}

/* Respond to incoming DMA PING messages with a PONG — on the same path
 * the PING arrived on.  rx_callback records the arrival path in the
 * verify_ping_mask bitmap; pings on different paths therefore can't
 * clobber each other even when they race. */
static void odl_tb5_ctrl_reply_work_fn(struct work_struct *work)
{
	struct odl_tb5_device *dev =
		container_of(work, struct odl_tb5_device, ctrl_reply_work);
	unsigned long mask;
	int p;

	if (atomic_read(&dev->removing))
		return;

	mask = (unsigned long)atomic_xchg(&dev->verify_ping_mask, 0);

	for_each_set_bit(p, &mask, ODL_TB5_MAX_PATHS) {
		if (p == 0)
			pr_info("OdinLink: DMA ping received, sending pong\n");
		else
			pr_info("OdinLink: DMA ping received on path %d, "
				"sending pong\n", p);
		odl_tb5_send_dma_msg(dev, ODL_TB5_DMA_PONG, p);
	}
}


/* Post-connection DMA verification via ping/pong exchange.
 *
 * IMPORTANT: We must NOT stop/start the RX ring between attempts.
 * Each ring stop cancels all pending RX frames, creating a dead
 * window where incoming PONGs are lost.  Instead, post enough RX
 * frames up front and just keep sending PINGs until a PONG arrives.
 */
static void odl_tb5_verify_work_fn(struct work_struct *work)
{
	struct odl_tb5_device *dev =
		container_of(work, struct odl_tb5_device, verify_work);
	int negotiated = dev->negotiated_paths;
	int verified = 0;
	long ret;
	int attempt, p;

	if (atomic_read(&dev->removing))
		return;

	if (negotiated < 1)
		negotiated = 1;

	atomic_set(&dev->pong_mask, 0);

	/* Use frame pool for RX during verify — each pool slot is
	 * independent and auto-reposts after consumption, so we never
	 * run out of RX frames.  The legacy submit_rx only posts 16
	 * frames and can't repost without a ring reset.  The RX window
	 * (pool/2) is split evenly across the negotiated paths. */
	if (dev->frame_pool.slots) {
		int per_path = (dev->frame_pool.size / 2) / negotiated;

		for (p = 0; p < negotiated; p++) {
			dev->paths[p].rx_target = per_path;
			odl_tb5_rx_repost(dev, p);
		}
		pr_info("OdinLink: verify using pool RX (target=%d)\n",
			dev->paths[0].rx_target);
	} else {
		size_t buf_size = (size_t)ODL_TB5_FRAME_SIZE * 16;

		ret = odl_tb5_submit_rx(dev, 0, buf_size);
		if (ret) {
			pr_warn("OdinLink: DMA verify: failed to post RX "
				"(%ld)\n", ret);
			goto out_reset;
		}
	}

	/* Verify every negotiated path sequentially, path 0 first.  Path 0
	 * dead => hard fail (as before).  Path i > 0 dead => degrade the
	 * TX side to the verified prefix; the path stays started/enabled
	 * on the RX side so an asymmetric peer can still reach us. */
	for (p = 0; p < negotiated; p++) {
		int max_attempts = (p == 0) ? 300 : 50;
		bool pong = false;

		for (attempt = 0; attempt < max_attempts; attempt++) {
			if (dev->state != ODL_TB5_STATE_CONNECTED)
				goto out_reset;

			flush_work(&dev->ctrl_reply_work);

			ret = odl_tb5_send_dma_msg(dev, ODL_TB5_DMA_PING, p);
			if (ret) {
				pr_warn("OdinLink: DMA verify: send ping failed "
					"(%ld)\n", ret);
				if (p == 0)
					goto out_reset;
				break;
			}

			if (attempt == 0) {
				if (p == 0)
					pr_info("OdinLink: DMA ping sent, "
						"waiting for pong\n");
				else
					pr_info("OdinLink: DMA ping sent on "
						"path %d, waiting for pong\n",
						p);
			}

			ret = wait_event_interruptible_timeout(
					dev->verify_waitq,
					atomic_read(&dev->pong_mask) & BIT(p),
					msecs_to_jiffies(100));
			if (ret > 0) {
				pong = true;
				break;
			}

			if (ret < 0)
				goto out_reset;

			if ((attempt + 1) % 50 == 0)
				pr_info("OdinLink: DMA ping attempt %d, "
					"still waiting for pong\n", attempt + 1);

			/* Do NOT reset the RX ring here — that kills in-flight
			 * frames and creates dead windows where PONGs are lost. */
		}

		if (!pong) {
			if (p == 0) {
				pr_err("OdinLink: DMA verify failed after %d attempts\n",
				       attempt);
				goto out_reset;
			}
			pr_warn("OdinLink: DMA verify failed on path %d, "
				"degrading TX to %d path(s)\n", p, p);
			break;
		}

		verified++;
	}

	dev->tx_active_paths = verified;

	pr_info("OdinLink: DMA path verified, resetting rings for userspace\n");
	if (verified > 1)
		pr_info("OdinLink: %d TX paths active\n", verified);

	flush_work(&dev->ctrl_reply_work);
	odl_tb5_poll_disarm(dev);
	odl_tb5_rings_reset(dev);

	mutex_lock(&dev->state_lock);
	dev->state = ODL_TB5_STATE_READY;
	wake_up_all(&dev->state_waitq);
	mutex_unlock(&dev->state_lock);

	/*
	 * Don't post pool frames yet — legacy consumers (daemon, CLI)
	 * don't use stream headers.  Pool RX repost starts when the
	 * first stream is opened via STREAM_OPEN ioctl.
	 *
	 * Do NOT force rx_posted to 0 here: the ring reset above cancels
	 * all posted pool frames and each cancel callback already
	 * decrements rx_posted.  Zeroing it as well double-accounts the
	 * cancels that arrive afterwards, driving rx_posted negative —
	 * the next repost then overshoots by that amount and starves the
	 * frame pool below the TX reserve (handshake sends block forever).
	 * This cancel-accounting applies per path: each path's cancel
	 * callbacks decrement that path's rx_posted.
	 */
	for (p = 0; p < ODL_TB5_MAX_PATHS; p++)
		dev->paths[p].rx_target = 0;

	/* Arm the on-demand poll for stream data — NHI MSI-X interrupts
	 * fire but descriptor write-back can lag, so we poll at 10 us to
	 * keep latency low.  It self-disarms once the device goes idle. */
	odl_tb5_poll_kick(dev);

	pr_info("OdinLink: entering READY state\n");
	return;

out_reset:
	odl_tb5_poll_disarm(dev);
	odl_tb5_rings_reset(dev);

	/* Liveness: a failed path-0 verify used to leave the device in
	 * CONNECTED limbo forever (peer half-connected after staggered
	 * reloads).  Restart the handshake instead; login retries have
	 * their own backoff if the peer is really gone. */
	if (dev->state == ODL_TB5_STATE_CONNECTED &&
	    !atomic_read(&dev->removing)) {
		int p;

		pr_warn("OdinLink: verify failed — restarting handshake\n");
		mutex_lock(&dev->state_lock);
		dev->login_sent = false;
		dev->login_received = false;
		/* restart_work disables/releases each path using
		 * stale_remote_tx_hopid.  On a verify-failure restart the
		 * paths are still enabled with the CURRENT remote hopids
		 * (no peer change happened), so seed stale from remote —
		 * otherwise the teardown runs with hopid 0, leaves the real
		 * router paths enabled and leaks the in-hopids, corrupting
		 * the DMA plane until a reboot. */
		for (p = 0; p < ODL_TB5_MAX_PATHS; p++)
			dev->paths[p].stale_remote_tx_hopid =
				dev->paths[p].remote_tx_hopid;
		mutex_unlock(&dev->state_lock);
		schedule_work(&dev->restart_work);
	}
}

/* Tear down stale connection and restart the handshake. */
static void odl_tb5_restart_work_fn(struct work_struct *work)
{
	struct odl_tb5_device *dev =
		container_of(work, struct odl_tb5_device, restart_work);
	int p;

	if (atomic_read(&dev->removing))
		return;

	odl_tb5_poll_disarm(dev);
	cancel_work_sync(&dev->verify_work);
	cancel_work_sync(&dev->ctrl_reply_work);
	cancel_work_sync(&dev->connect_work);
	cancel_delayed_work_sync(&dev->login_work);
	atomic_set(&dev->pong_mask, 0);
	atomic_set(&dev->verify_ping_mask, 0);
	dev->tx_active_paths = 1;

	/* M1 ordering, now per path: first disable ALL router paths, then
	 * stop the rings, then release the input HopIDs (guarded by
	 * in_hopid_valid against double release). */
	for (p = 0; p < dev->num_paths; p++) {
		struct odl_tb5_path *path = &dev->paths[p];

		if (path->tx.started)
			tb_xdomain_disable_paths(dev->xd,
						 path->local_tx_hopid,
						 path->tx.ring->hop,
						 path->stale_remote_tx_hopid,
						 path->rx.ring->hop);
	}

	odl_tb5_rings_stop(dev);

	for (p = 0; p < dev->num_paths; p++) {
		struct odl_tb5_path *path = &dev->paths[p];

		if (path->in_hopid_valid) {
			tb_xdomain_release_in_hopid(dev->xd,
						    path->stale_remote_tx_hopid);
			path->in_hopid_valid = false;
		}
	}

	mutex_lock(&dev->state_lock);
	dev->state = ODL_TB5_STATE_HANDSHAKE;
	wake_up_all(&dev->state_waitq);
	dev->login_sent = false;
	dev->login_retries = 0;
	mutex_unlock(&dev->state_lock);

	pr_info("OdinLink: connection restarted, beginning handshake\n");
	if (!atomic_read(&dev->removing))
		schedule_delayed_work(&dev->login_work, 0);
}

/* Delayed work handler that retries login with exponential backoff. */
static void odl_tb5_login_work_fn(struct work_struct *work)
{
	struct odl_tb5_device *dev =
		container_of(work, struct odl_tb5_device, login_work.work);
	unsigned long delay_ms;
	int ret;

	if (atomic_read(&dev->removing))
		return;

	/* The liveness re-login (see send_login) may fire after the
	 * handshake completed — don't clobber negotiated state then. */
	mutex_lock(&dev->state_lock);
	if (dev->state != ODL_TB5_STATE_HANDSHAKE) {
		mutex_unlock(&dev->state_lock);
		return;
	}
	mutex_unlock(&dev->state_lock);

	ret = odl_tb5_proto_send_login(dev);
	if (ret && !atomic_read(&dev->removing)) {
		dev->login_retries++;

		delay_ms = ODL_TB5_LOGIN_TIMEOUT <<
			   min_t(int, dev->login_retries, 4);
		if (delay_ms > 5000)
			delay_ms = 5000;

		if (dev->login_retries <= 5 ||
		    dev->login_retries % 10 == 0)
			pr_info("OdinLink: login attempt %d failed (%d), "
				"retrying in %lu ms\n",
				dev->login_retries, ret, delay_ms);

		schedule_delayed_work(&dev->login_work,
				      msecs_to_jiffies(delay_ms));
	}
}

/* Send a logout notification to the peer. */
int odl_tb5_proto_send_logout(struct odl_tb5_device *dev)
{
	struct odl_tb5_logout_msg msg = { };
	struct odl_tb5_xd_header resp = { };

	odl_tb5_xd_header_init(&msg.xd_hdr, dev->xd, ODL_TB5_MSG_LOGOUT,
			       sizeof(msg));

	tb_xdomain_request(dev->xd, &msg, sizeof(msg),
			   TB_CFG_PKG_XDOMAIN_REQ,
			   &resp, sizeof(resp),
			   TB_CFG_PKG_XDOMAIN_RESP,
			   ODL_TB5_LOGIN_TIMEOUT);

	mutex_lock(&dev->state_lock);
	dev->state = ODL_TB5_STATE_DISCONNECTED;
	wake_up_all(&dev->state_waitq);
	mutex_unlock(&dev->state_lock);

	pr_info("OdinLink: logout sent to peer\n");

	return 0;
}

/* Initialise the protocol layer for a device. */
int odl_tb5_proto_init(struct odl_tb5_device *dev)
{
	INIT_DELAYED_WORK(&dev->login_work, odl_tb5_login_work_fn);
	INIT_WORK(&dev->connect_work, odl_tb5_connect_work_fn);
	INIT_WORK(&dev->restart_work, odl_tb5_restart_work_fn);
	INIT_WORK(&dev->verify_work, odl_tb5_verify_work_fn);
	INIT_WORK(&dev->ctrl_reply_work, odl_tb5_ctrl_reply_work_fn);
	hrtimer_setup(&dev->rx_poll_timer, odl_tb5_rx_poll_timer_fn,
		      CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	atomic_set(&dev->poll_active, 0);
	atomic_set(&dev->tx_inflight, 0);
	dev->poll_last_rxseen = 0;
	dev->poll_idle_ticks = 0;
	init_waitqueue_head(&dev->verify_waitq);

	dev->login_retries  = 0;
	dev->login_sent     = false;
	dev->login_received = false;
	atomic_set(&dev->pong_mask, 0);
	atomic_set(&dev->verify_ping_mask, 0);

	mutex_lock(&dev->state_lock);
	dev->state = ODL_TB5_STATE_HANDSHAKE;
	wake_up_all(&dev->state_waitq);
	mutex_unlock(&dev->state_lock);

	schedule_delayed_work(&dev->login_work, 0);

	return 0;
}

/* Tear down the protocol layer for a device. */
void odl_tb5_proto_exit(struct odl_tb5_device *dev)
{
	odl_tb5_poll_disarm(dev);
	cancel_work_sync(&dev->verify_work);
	cancel_work_sync(&dev->ctrl_reply_work);
	cancel_work_sync(&dev->restart_work);
	cancel_work_sync(&dev->connect_work);
	cancel_delayed_work_sync(&dev->login_work);
}
