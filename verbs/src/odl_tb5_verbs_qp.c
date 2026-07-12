/*
 * OdinLink — Verbs: Queue Pairs (The Actual Data Path)
 *
 * A QP maps to an OdinLink stream.  On top of the stream's two-sided,
 * message-framed SEND/RECV we emulate the full RC verb set — SEND/RECV,
 * RDMA WRITE, RDMA WRITE_WITH_IMM and RDMA READ — by prefixing every message
 * with a small odl_rdma_hdr (see odl_tb5_verbs.h).  Each verb is one header
 * message optionally followed by one payload message; because the transport
 * preserves message boundaries, the responder can read the header first and
 * dispatch on the opcode.  This is the "software RDMA" pattern (à la Soft-RoCE
 * / libfabric's socket providers): one-sided ops become a control header plus
 * a data frame that the peer's recv worker services.
 *
 * Threading:
 *   send worker (odl_qp_worker)      — drains the SQ of odl_tx_desc, and is the
 *                                      ONLY thread that transmits on the stream
 *                                      (so the [hdr][payload] pair is atomic).
 *   recv worker (odl_qp_recv_worker) — reads incoming headers and dispatches:
 *                                      places WRITE data, answers READ_REQ by
 *                                      enqueuing a READ_RESP onto the SQ,
 *                                      matches READ_RESP, delivers SEND to an
 *                                      RQ WR.  It never transmits directly.
 *
 * Both workers busy-poll with sched_yield(): the device fd is O_NONBLOCK and
 * the driver does not raise EPOLLIN/OUT for stream traffic, so a blocking poll
 * would stall for its full timeout on every op.
 */

#include "odl_tb5_verbs.h"
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sched.h>

/* ── Small helpers ──────────────────────────────────────────────────── */

/* Look up a dmabuf fd by lkey. Returns -1 if not a dmabuf MR. */
static int odl_lookup_dmabuf(struct odl_verbs_context *ctx, uint32_t lkey)
{
    struct odl_verbs_mr *mr = odl_find_mr_by_rkey(ctx, lkey);
    return (mr && mr->mr_type == 1) ? mr->dmabuf_fd : -1;
}

/* Transmit one stream message, retrying while the non-blocking fd is full.
 * Only ever called from the send worker. */
static int tx_msg(struct odl_verbs_qp *qp, const void *buf, uint32_t len)
{
    odl_tb5_t h = qp->ctx->handle;
    for (;;) {
        int ret = odl_tb5_stream_send(h, qp->stream_id,
                                      qp->dest_stream_id, buf, len);
        if (ret != -EAGAIN)
            return ret;
        if (!qp->worker_running)
            return -EINTR;
        sched_yield();
    }
}

static int tx_dmabuf(struct odl_verbs_qp *qp, int fd,
                     uint64_t offset, uint32_t len)
{
    odl_tb5_t h = qp->ctx->handle;
    for (;;) {
        int ret = odl_tb5_stream_send_dmabuf(h, qp->stream_id,
                                             qp->dest_stream_id,
                                             fd, offset, len);
        if (ret != -EAGAIN)
            return ret;
        if (!qp->worker_running)
            return -EINTR;
        sched_yield();
    }
}

/* Receive one whole stream message into buf, retrying until it arrives.
 * Only ever called from the recv worker (after a header committed us to a
 * payload frame that is already in flight). */
static int rx_msg(struct odl_verbs_qp *qp, void *buf, uint32_t buf_len,
                  uint32_t *actual)
{
    odl_tb5_t h = qp->ctx->handle;
    uint8_t src = 0;
    uint32_t act = 0;
    for (;;) {
        int ret = odl_tb5_stream_recv(h, qp->stream_id, buf, buf_len,
                                      &src, &act);
        if (ret != -EAGAIN) {
            if (actual) *actual = act;
            return ret;
        }
        if (!qp->recv_worker_running)
            return -EINTR;
        sched_yield();
    }
}

static int rx_dmabuf(struct odl_verbs_qp *qp, int fd,
                     uint64_t offset, uint32_t len)
{
    odl_tb5_t h = qp->ctx->handle;
    for (;;) {
        int ret = odl_tb5_stream_recv_dmabuf(h, qp->stream_id,
                                             fd, offset, len);
        if (ret != -EAGAIN)
            return ret;
        if (!qp->recv_worker_running)
            return -EINTR;
        sched_yield();
    }
}

/* Consume an incoming payload message we cannot place (unknown rkey): drain it
 * into a throwaway buffer so the stream stays in frame. */
static void rx_discard(struct odl_verbs_qp *qp, uint32_t len)
{
    if (len == 0) return;
    void *tmp = malloc(len);
    if (!tmp) {
        odl_logerr("rx_discard: OOM draining %u bytes — stream may desync", len);
        return;
    }
    rx_msg(qp, tmp, len, NULL);
    free(tmp);
}

/* Pop the head RQ WR, or NULL if none posted. */
static struct ibv_recv_wr *rq_pop(struct odl_verbs_qp *qp)
{
    struct ibv_recv_wr *rwr = NULL;
    pthread_mutex_lock(&qp->rq_lock);
    if (qp->rq_count > 0) {
        rwr = qp->rq[qp->rq_head];
        qp->rq_head = (qp->rq_head + 1) % ODL_VERBS_RQ_DEPTH;
        qp->rq_count--;
    }
    pthread_mutex_unlock(&qp->rq_lock);
    return rwr;
}

/* Wait for an RQ WR to be posted (RNR-style flow control). */
static struct ibv_recv_wr *rq_pop_wait(struct odl_verbs_qp *qp)
{
    for (;;) {
        struct ibv_recv_wr *rwr = rq_pop(qp);
        if (rwr) return rwr;
        if (!qp->recv_worker_running) return NULL;
        sched_yield();
    }
}

/* Enqueue an internal TX descriptor onto the SQ (used by the recv worker to
 * hand a READ_RESP to the send worker).  Waits for space if the SQ is full. */
static void sq_push(struct odl_verbs_qp *qp, const struct odl_tx_desc *d)
{
    for (;;) {
        pthread_mutex_lock(&qp->sq_lock);
        if (qp->sq_count < ODL_VERBS_SQ_DEPTH) {
            qp->sq[qp->sq_tail] = *d;
            qp->sq_tail = (qp->sq_tail + 1) % ODL_VERBS_SQ_DEPTH;
            qp->sq_count++;
            pthread_mutex_unlock(&qp->sq_lock);
            return;
        }
        pthread_mutex_unlock(&qp->sq_lock);
        if (!qp->recv_worker_running) return;
        sched_yield();
    }
}

/* ── Send worker: drain the SQ, transmit [hdr][payload] ─────────────── */

static void tx_execute(struct odl_verbs_qp *qp, struct odl_tx_desc *d)
{
    struct odl_rdma_hdr hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.magic       = ODL_RDMA_MAGIC;
    hdr.op          = d->op;
    hdr.length      = d->length;
    hdr.imm_data    = d->imm_data;
    hdr.remote_addr = d->remote_addr;
    hdr.rkey        = d->rkey;
    hdr.read_id     = d->read_id;

    int ret = tx_msg(qp, &hdr, (uint32_t)sizeof(hdr));

    /* Payload frame follows for everything except a READ request (whose
     * length names the bytes to fetch, not bytes carried here). */
    if (ret == 0 && d->length > 0 && d->op != ODL_OP_READ_REQ) {
        if (d->dmabuf_fd >= 0)
            ret = tx_dmabuf(qp, d->dmabuf_fd, d->dmabuf_offset, d->length);
        else if (d->host_addr)
            ret = tx_msg(qp, d->host_addr, d->length);
    }

    if (ret != 0 && ret != -EINTR)
        odl_logerr("TX op=%u stream=%u dst=%u ret=%d len=%u",
                   d->op, qp->stream_id, qp->dest_stream_id, ret, d->length);
    else
        odl_logverbose("TX op=%u stream=%u dst=%u ret=%d len=%u",
                       d->op, qp->stream_id, qp->dest_stream_id, ret, d->length);

    if (d->free_host && d->host_addr)
        free(d->host_addr);

    /* READ_REQ: completion is deferred until the READ_RESP lands (posted by
     * the recv worker).  READ_RESP: internal, never completes locally. */
    if (d->op == ODL_OP_READ_REQ || d->op == ODL_OP_READ_RESP)
        return;

    if (d->signaled && d->cq) {
        struct ibv_wc wc;
        memset(&wc, 0, sizeof(wc));
        wc.wr_id    = d->wr_id;
        wc.qp_num   = qp->base.qp_num;
        wc.opcode   = d->wc_opcode;
        wc.status   = (ret == 0) ? IBV_WC_SUCCESS : IBV_WC_GENERAL_ERR;
        wc.byte_len = (ret == 0) ? d->length : 0;
        odl_cq_post(d->cq, &wc);
    }
    atomic_fetch_sub(&qp->pending_sends, 1);
}

static void *odl_qp_worker(void *arg)
{
    struct odl_verbs_qp *qp = arg;

    while (qp->worker_running) {
        struct odl_tx_desc d;
        bool have = false;

        pthread_mutex_lock(&qp->sq_lock);
        if (qp->sq_count > 0) {
            d = qp->sq[qp->sq_head];
            qp->sq_head = (qp->sq_head + 1) % ODL_VERBS_SQ_DEPTH;
            qp->sq_count--;
            have = true;
        }
        pthread_mutex_unlock(&qp->sq_lock);

        if (!have) {
            sched_yield();
            continue;
        }
        tx_execute(qp, &d);
    }
    return NULL;
}

/* ── Recv worker: read a header, service the op ─────────────────────── */

/* SEND → deliver the payload into a posted RQ WR and post IBV_WC_RECV. */
static void handle_send(struct odl_verbs_qp *qp, struct odl_rdma_hdr *hdr)
{
    struct odl_verbs_context *ctx = qp->ctx;
    struct ibv_recv_wr *rwr = rq_pop_wait(qp);
    if (!rwr) { rx_discard(qp, hdr->length); return; }

    struct ibv_sge *sge = (rwr->num_sge > 0) ? &rwr->sg_list[0] : NULL;
    int ret = 0;
    uint32_t actual = 0;

    if (sge && hdr->length > 0) {
        int fd = odl_lookup_dmabuf(ctx, sge->lkey);
        if (fd >= 0) {
            ret = rx_dmabuf(qp, fd, 0, hdr->length);
            actual = (ret == 0) ? hdr->length : 0;
        } else {
            ret = rx_msg(qp, (void *)(uintptr_t)sge->addr, hdr->length, &actual);
        }
    } else if (hdr->length > 0) {
        rx_discard(qp, hdr->length);
    }

    struct ibv_wc wc;
    memset(&wc, 0, sizeof(wc));
    wc.wr_id    = rwr->wr_id;
    wc.qp_num   = qp->base.qp_num;
    wc.src_qp   = qp->dest_stream_id;
    wc.opcode   = IBV_WC_RECV;
    wc.status   = (ret == 0) ? IBV_WC_SUCCESS : IBV_WC_GENERAL_ERR;
    wc.byte_len = actual;
    if (qp->recv_cq)
        odl_cq_post(qp->recv_cq, &wc);
    atomic_fetch_sub(&qp->pending_recvs, 1);
}

/* WRITE / WRITE_IMM → place the payload directly at the peer-named address. */
static void handle_write(struct odl_verbs_qp *qp, struct odl_rdma_hdr *hdr)
{
    struct odl_verbs_context *ctx = qp->ctx;
    struct odl_verbs_mr *mr = odl_find_mr_by_rkey(ctx, hdr->rkey);
    int ret = 0;

    if (hdr->length > 0) {
        if (mr && mr->mr_type == 1) {
            /* Zero-copy GPU dest: offset of remote_addr within the dmabuf. */
            uint64_t off = (hdr->remote_addr >= mr->iova)
                         ? hdr->remote_addr - mr->iova : 0;
            ret = rx_dmabuf(qp, mr->dmabuf_fd, off, hdr->length);
        } else if (mr) {
            /* Host dest: remote_addr is a valid pointer into our own MR. */
            ret = rx_msg(qp, (void *)(uintptr_t)hdr->remote_addr,
                         hdr->length, NULL);
        } else {
            odl_logerr("WRITE to unknown rkey=%08x — discarding %u bytes",
                       hdr->rkey, hdr->length);
            rx_discard(qp, hdr->length);
            ret = -EINVAL;
        }
    }

    /* WRITE_WITH_IMM raises a receive completion carrying the immediate. */
    if (hdr->op == ODL_OP_WRITE_IMM) {
        struct ibv_recv_wr *rwr = rq_pop_wait(qp);
        struct ibv_wc wc;
        memset(&wc, 0, sizeof(wc));
        wc.wr_id    = rwr ? rwr->wr_id : 0;
        wc.qp_num   = qp->base.qp_num;
        wc.src_qp   = qp->dest_stream_id;
        wc.opcode   = IBV_WC_RECV_RDMA_WITH_IMM;
        wc.status   = (ret == 0) ? IBV_WC_SUCCESS : IBV_WC_GENERAL_ERR;
        wc.byte_len = hdr->length;
        wc.wc_flags = IBV_WC_WITH_IMM;
        wc.imm_data = hdr->imm_data;
        if (qp->recv_cq)
            odl_cq_post(qp->recv_cq, &wc);
        if (rwr)
            atomic_fetch_sub(&qp->pending_recvs, 1);
    }
}

/* READ_REQ → fetch from our local MR and reply with a READ_RESP + payload. */
static void handle_read_req(struct odl_verbs_qp *qp, struct odl_rdma_hdr *hdr)
{
    struct odl_verbs_context *ctx = qp->ctx;
    struct odl_verbs_mr *mr = odl_find_mr_by_rkey(ctx, hdr->rkey);

    struct odl_tx_desc d;
    memset(&d, 0, sizeof(d));
    d.op        = ODL_OP_READ_RESP;
    d.length    = hdr->length;
    d.read_id   = hdr->read_id;
    d.dmabuf_fd = -1;

    if (mr && mr->mr_type == 1) {
        d.dmabuf_fd     = mr->dmabuf_fd;
        d.dmabuf_offset = (hdr->remote_addr >= mr->iova)
                        ? hdr->remote_addr - mr->iova : 0;
    } else if (mr) {
        /* Read straight out of our own registered buffer. */
        d.host_addr = (void *)(uintptr_t)hdr->remote_addr;
    } else {
        /* Unknown rkey: reply with zeros so the initiator doesn't hang. */
        odl_logerr("READ from unknown rkey=%08x — replying zeros (%u bytes)",
                   hdr->rkey, hdr->length);
        d.host_addr = calloc(1, hdr->length ? hdr->length : 1);
        d.free_host = true;
    }
    sq_push(qp, &d);
}

/* READ_RESP → deliver fetched bytes into the initiator's buffer and complete
 * the pending IBV_WR_RDMA_READ. */
static void handle_read_resp(struct odl_verbs_qp *qp, struct odl_rdma_hdr *hdr)
{
    struct odl_read_pending pend;
    bool matched = false;

    pthread_mutex_lock(&qp->rp_lock);
    if (qp->rp_count > 0) {
        pend = qp->rp[qp->rp_head];
        qp->rp_head = (qp->rp_head + 1) % ODL_VERBS_SQ_DEPTH;
        qp->rp_count--;
        matched = true;
    }
    pthread_mutex_unlock(&qp->rp_lock);

    if (!matched) {
        odl_logerr("READ_RESP read_id=%llu with no pending read — discarding",
                   (unsigned long long)hdr->read_id);
        rx_discard(qp, hdr->length);
        return;
    }
    if (pend.read_id != hdr->read_id)
        odl_logerr("READ_RESP id mismatch: got %llu expected %llu",
                   (unsigned long long)hdr->read_id,
                   (unsigned long long)pend.read_id);

    int ret = 0;
    uint32_t actual = 0;
    if (hdr->length > 0) {
        if (pend.dmabuf_fd >= 0) {
            ret = rx_dmabuf(qp, pend.dmabuf_fd, pend.dmabuf_offset, hdr->length);
            actual = (ret == 0) ? hdr->length : 0;
        } else if (pend.host_addr) {
            ret = rx_msg(qp, pend.host_addr, hdr->length, &actual);
        }
    }

    if (pend.signaled && pend.cq) {
        struct ibv_wc wc;
        memset(&wc, 0, sizeof(wc));
        wc.wr_id    = pend.wr_id;
        wc.qp_num   = qp->base.qp_num;
        wc.opcode   = IBV_WC_RDMA_READ;
        wc.status   = (ret == 0) ? IBV_WC_SUCCESS : IBV_WC_GENERAL_ERR;
        wc.byte_len = actual;
        odl_cq_post(pend.cq, &wc);
    }
    atomic_fetch_sub(&qp->pending_sends, 1);
}

static void *odl_qp_recv_worker(void *arg)
{
    struct odl_verbs_qp *qp = arg;
    odl_tb5_t h = qp->ctx->handle;

    while (qp->recv_worker_running) {
        struct odl_rdma_hdr hdr;
        uint8_t src = 0;
        uint32_t actual = 0;

        int ret = odl_tb5_stream_recv(h, qp->stream_id, &hdr, sizeof(hdr),
                                      &src, &actual);
        if (ret == -EAGAIN) {
            sched_yield();
            continue;
        }
        if (ret != 0) {
            odl_logverbose("RX hdr stream=%u ret=%d", qp->stream_id, ret);
            sched_yield();
            continue;
        }
        if (actual < sizeof(hdr) || hdr.magic != ODL_RDMA_MAGIC) {
            odl_logerr("RX bad header stream=%u actual=%u magic=%08x",
                       qp->stream_id, actual, hdr.magic);
            continue;
        }

        switch (hdr.op) {
        case ODL_OP_SEND:      handle_send(qp, &hdr);      break;
        case ODL_OP_WRITE:
        case ODL_OP_WRITE_IMM: handle_write(qp, &hdr);     break;
        case ODL_OP_READ_REQ:  handle_read_req(qp, &hdr);  break;
        case ODL_OP_READ_RESP: handle_read_resp(qp, &hdr); break;
        default:
            odl_logerr("RX unknown op=%u stream=%u", hdr.op, qp->stream_id);
            break;
        }
    }
    return NULL;
}

/* ── Create QP ──────────────────────────────────────────────────────── */

struct ibv_qp *odl_create_qp(struct ibv_pd *pd,
                              struct ibv_qp_init_attr *attr)
{
    ODL_TRACE_ENTRY();
    ODL_RETURN_NULL_IF(!pd || !attr, "null argument");

    struct odl_verbs_context *ctx = odl_ctx_from_ibv(pd->context);

    if (attr->qp_type != IBV_QPT_RC) {
        odl_logerr("unsupported QP type: %d", attr->qp_type);
        errno = EOPNOTSUPP;
        return NULL;
    }

    struct odl_verbs_qp *qp = calloc(1, sizeof(*qp));
    if (!qp) { errno = ENOMEM; return NULL; }

    /* Open an OdinLink-Five stream */
    uint8_t stream_id = 0;
    int ret = odl_tb5_stream_open(ctx->handle, 0, &stream_id);
    if (ret != 0) {
        odl_logerr("stream_open failed: %d", ret);
        free(qp);
        errno = ENODEV;
        return NULL;
    }

    qp->base.context     = pd->context;
    qp->base.pd          = pd;
    qp->base.send_cq     = attr->send_cq;
    qp->base.recv_cq     = attr->recv_cq;
    qp->base.qp_num      = stream_id;
    qp->base.qp_type     = attr->qp_type;
    qp->base.state       = IBV_QPS_RESET;
    qp->ctx              = ctx;
    qp->pd               = odl_pd_from_ibv(pd);
    qp->send_cq          = attr->send_cq ? odl_cq_from_ibv(attr->send_cq) : NULL;
    qp->recv_cq          = attr->recv_cq ? odl_cq_from_ibv(attr->recv_cq) : NULL;
    qp->stream_id        = stream_id;

    pthread_mutex_init(&qp->sq_lock, NULL);
    qp->sq_head = qp->sq_tail = qp->sq_count = 0;

    pthread_mutex_init(&qp->rq_lock, NULL);
    qp->rq_head = qp->rq_tail = qp->rq_count = 0;

    pthread_mutex_init(&qp->rp_lock, NULL);
    qp->rp_head = qp->rp_tail = qp->rp_count = 0;
    atomic_init(&qp->read_id_next, 1);

    atomic_init(&qp->pending_sends, 0);
    atomic_init(&qp->pending_recvs, 0);

    /* Track in context */
    pthread_mutex_lock(&ctx->qp_lock);
    if (ctx->nqps >= ODL_VERBS_MAX_QPS) {
        pthread_mutex_unlock(&ctx->qp_lock);
        odl_tb5_stream_close(ctx->handle, stream_id);
        pthread_mutex_destroy(&qp->sq_lock);
        pthread_mutex_destroy(&qp->rq_lock);
        pthread_mutex_destroy(&qp->rp_lock);
        free(qp);
        errno = ENOMEM;
        return NULL;
    }
    ctx->qps[ctx->nqps++] = qp;
    pthread_mutex_unlock(&ctx->qp_lock);

    /* Start async worker threads: one for sends, one for receives. */
    qp->worker_running = true;
    ret = pthread_create(&qp->worker, NULL, odl_qp_worker, qp);
    if (ret != 0) {
        odl_logerr("pthread_create (send) failed: %d", ret);
        qp->worker_running = false;
        odl_tb5_stream_close(ctx->handle, stream_id);
        pthread_mutex_destroy(&qp->sq_lock);
        pthread_mutex_destroy(&qp->rq_lock);
        pthread_mutex_destroy(&qp->rp_lock);
        free(qp);
        errno = EAGAIN;
        return NULL;
    }

    qp->recv_worker_running = true;
    ret = pthread_create(&qp->recv_worker, NULL, odl_qp_recv_worker, qp);
    if (ret != 0) {
        odl_logerr("pthread_create (recv) failed: %d", ret);
        qp->recv_worker_running = false;
        qp->worker_running = false;
        pthread_join(qp->worker, NULL);
        odl_tb5_stream_close(ctx->handle, stream_id);
        pthread_mutex_destroy(&qp->sq_lock);
        pthread_mutex_destroy(&qp->rq_lock);
        pthread_mutex_destroy(&qp->rp_lock);
        free(qp);
        errno = EAGAIN;
        return NULL;
    }

    odl_loginfo("create_qp: stream=%u qp_num=%u", stream_id, qp->base.qp_num);
    ODL_TRACE_EXIT();
    return &qp->base;
}

/* ── Destroy QP ─────────────────────────────────────────────────────── */

int odl_destroy_qp(struct ibv_qp *qp)
{
    ODL_TRACE_ENTRY();
    if (!qp) return -EINVAL;

    struct odl_verbs_qp *oqp = odl_qp_from_ibv(qp);
    struct odl_verbs_context *ctx = oqp->ctx;

    oqp->worker_running = false;
    oqp->recv_worker_running = false;
    pthread_join(oqp->worker, NULL);
    pthread_join(oqp->recv_worker, NULL);

    if (oqp->stream_id > 0)
        odl_tb5_stream_close(ctx->handle, oqp->stream_id);

    pthread_mutex_lock(&ctx->qp_lock);
    for (int i = 0; i < ctx->nqps; i++) {
        if (ctx->qps[i] == oqp) {
            ctx->qps[i] = ctx->qps[--ctx->nqps];
            break;
        }
    }
    pthread_mutex_unlock(&ctx->qp_lock);

    pthread_mutex_destroy(&oqp->sq_lock);
    pthread_mutex_destroy(&oqp->rq_lock);
    pthread_mutex_destroy(&oqp->rp_lock);
    free(oqp);

    odl_loginfo("destroy_qp done");
    ODL_TRACE_EXIT_VAL(0);
}

/* ── Modify QP ─────────────────────────────────────────────────────── */

int odl_modify_qp(struct ibv_qp *qp, struct ibv_qp_attr *attr,
                   int attr_mask)
{
    ODL_TRACE_ENTRY();
    ODL_RETURN_EINVAL_IF(!qp, "null qp");

    /* Capture the peer's stream id from the RDMA connection handshake.
     * qp_num == stream_id on both ends, so dest_qp_num is the peer's stream. */
    if (attr_mask & IBV_QP_DEST_QPN) {
        struct odl_verbs_qp *oqp = odl_qp_from_ibv(qp);
        oqp->dest_stream_id = (uint8_t)attr->dest_qp_num;
        odl_loginfo("modify_qp: qp_num=%u dest_stream=%u",
                     qp->qp_num, oqp->dest_stream_id);
    }

    if (attr_mask & IBV_QP_STATE) {
        qp->state = attr->qp_state;
        odl_loginfo("modify_qp: qp_num=%u state -> %d",
                     qp->qp_num, attr->qp_state);

        if (attr->qp_state == IBV_QPS_RTS) {
            struct odl_verbs_qp *oqp = odl_qp_from_ibv(qp);
            odl_tb5_wait_peer(oqp->ctx->handle, 5000);
        }
    }

    ODL_TRACE_EXIT_VAL(0);
}

/* ── Post Send (async) ──────────────────────────────────────────────── */

int odl_post_send(struct ibv_qp *qp, struct ibv_send_wr *wr,
                   struct ibv_send_wr **bad_wr)
{
    struct odl_verbs_qp *oqp = odl_qp_from_ibv(qp);
    struct odl_verbs_context *ctx = oqp->ctx;
    *bad_wr = NULL;

    while (wr) {
        struct ibv_sge *sge = (wr->num_sge > 0) ? &wr->sg_list[0] : NULL;
        uint32_t len = sge ? sge->length : 0;

        struct odl_tx_desc d;
        memset(&d, 0, sizeof(d));
        d.wr_id     = wr->wr_id;
        d.length    = len;
        d.dmabuf_fd = -1;
        d.signaled  = (wr->send_flags & IBV_SEND_SIGNALED) != 0;
        d.cq        = oqp->send_cq;

        /* Payload source: host pointer or zero-copy dmabuf (by lkey). */
        if (sge) {
            int fd = odl_lookup_dmabuf(ctx, sge->lkey);
            if (fd >= 0)
                d.dmabuf_fd = fd;      /* offset 0 == whole registered buffer */
            else
                d.host_addr = (void *)(uintptr_t)sge->addr;
        }

        switch (wr->opcode) {
        case IBV_WR_SEND:
            d.op = ODL_OP_SEND;
            d.wc_opcode = IBV_WC_SEND;
            break;
        case IBV_WR_RDMA_WRITE:
            d.op = ODL_OP_WRITE;
            d.wc_opcode = IBV_WC_RDMA_WRITE;
            d.remote_addr = wr->wr.rdma.remote_addr;
            d.rkey        = wr->wr.rdma.rkey;
            break;
        case IBV_WR_RDMA_WRITE_WITH_IMM:
            d.op = ODL_OP_WRITE_IMM;
            d.wc_opcode = IBV_WC_RDMA_WRITE;
            d.remote_addr = wr->wr.rdma.remote_addr;
            d.rkey        = wr->wr.rdma.rkey;
            d.imm_data    = wr->imm_data;
            break;
        case IBV_WR_RDMA_READ: {
            /* Register the outstanding read, then send a READ_REQ.  The
             * READ_RESP will match this entry and complete the WR. */
            d.op = ODL_OP_READ_REQ;
            d.wc_opcode = IBV_WC_RDMA_READ;
            d.remote_addr = wr->wr.rdma.remote_addr;
            d.rkey        = wr->wr.rdma.rkey;
            d.read_id = atomic_fetch_add(&oqp->read_id_next, 1);

            struct odl_read_pending pend;
            memset(&pend, 0, sizeof(pend));
            pend.read_id  = d.read_id;
            pend.wr_id    = wr->wr_id;
            pend.cq       = oqp->send_cq;
            pend.signaled = d.signaled;
            pend.length   = len;
            pend.dmabuf_fd = d.dmabuf_fd;     /* local dest by lkey */
            pend.dmabuf_offset = 0;
            pend.host_addr = (d.dmabuf_fd < 0) ? d.host_addr : NULL;

            pthread_mutex_lock(&oqp->rp_lock);
            if (oqp->rp_count >= ODL_VERBS_SQ_DEPTH) {
                pthread_mutex_unlock(&oqp->rp_lock);
                *bad_wr = wr;
                odl_logerr("post_send: read-pending queue full on QP %u",
                           qp->qp_num);
                return -ENOMEM;
            }
            oqp->rp[oqp->rp_tail] = pend;
            oqp->rp_tail = (oqp->rp_tail + 1) % ODL_VERBS_SQ_DEPTH;
            oqp->rp_count++;
            pthread_mutex_unlock(&oqp->rp_lock);
            /* READ carries no payload frame; host_addr/dmabuf are the dest. */
            d.host_addr = NULL;
            break;
        }
        default:
            *bad_wr = wr;
            odl_logerr("post_send: unsupported opcode %d on QP %u",
                       wr->opcode, qp->qp_num);
            return -EOPNOTSUPP;
        }

        /* Enqueue the descriptor for the send worker. */
        pthread_mutex_lock(&oqp->sq_lock);
        if (oqp->sq_count >= ODL_VERBS_SQ_DEPTH) {
            pthread_mutex_unlock(&oqp->sq_lock);
            *bad_wr = wr;
            odl_logerr("post_send: SQ full on QP %u", qp->qp_num);
            return -ENOMEM;
        }
        oqp->sq[oqp->sq_tail] = d;
        oqp->sq_tail = (oqp->sq_tail + 1) % ODL_VERBS_SQ_DEPTH;
        oqp->sq_count++;
        atomic_fetch_add(&oqp->pending_sends, 1);
        pthread_mutex_unlock(&oqp->sq_lock);

        wr = wr->next;
    }
    return 0;
}

/* ── Post Recv (async: enqueue only, recv worker delivers) ──────────── */

int odl_post_recv(struct ibv_qp *qp, struct ibv_recv_wr *wr,
                   struct ibv_recv_wr **bad_wr)
{
    struct odl_verbs_qp *oqp = odl_qp_from_ibv(qp);
    *bad_wr = NULL;

    pthread_mutex_lock(&oqp->rq_lock);
    while (wr) {
        if (oqp->rq_count >= ODL_VERBS_RQ_DEPTH) {
            *bad_wr = wr;
            pthread_mutex_unlock(&oqp->rq_lock);
            odl_logerr("post_recv: RQ full on QP %u", qp->qp_num);
            return -ENOMEM;
        }

        oqp->rq[oqp->rq_tail] = wr;
        oqp->rq_tail = (oqp->rq_tail + 1) % ODL_VERBS_RQ_DEPTH;
        oqp->rq_count++;
        atomic_fetch_add(&oqp->pending_recvs, 1);

        wr = wr->next;
    }
    pthread_mutex_unlock(&oqp->rq_lock);
    return 0;
}

/* ── Query QP ───────────────────────────────────────────────────────── */

int odl_query_qp(struct ibv_qp *qp, struct ibv_qp_attr *attr,
                  int attr_mask, struct ibv_qp_init_attr *init_attr)
{
    ODL_TRACE_ENTRY();
    ODL_RETURN_EINVAL_IF(!qp, "null qp");
    (void)attr_mask;

    memset(attr, 0, sizeof(*attr));
    attr->qp_state         = qp->state;
    attr->cur_qp_state     = qp->state;
    attr->path_mtu         = IBV_MTU_4096;
    attr->path_mig_state   = IBV_MIG_MIGRATED;
    attr->qkey             = 0;
    attr->rq_psn           = 0;
    attr->sq_psn           = 0;
    attr->dest_qp_num      = 0;
    attr->qp_access_flags  = IBV_ACCESS_LOCAL_WRITE |
                              IBV_ACCESS_REMOTE_WRITE |
                              IBV_ACCESS_REMOTE_READ;
    attr->cap.max_send_wr  = ODL_VERBS_SQ_DEPTH;
    attr->cap.max_recv_wr  = ODL_VERBS_SQ_DEPTH;
    attr->cap.max_send_sge = 1;
    attr->cap.max_recv_sge = 1;
    attr->cap.max_inline_data = 0;
    attr->port_num         = 1;
    attr->timeout          = 0;
    attr->retry_cnt        = 0;
    attr->rnr_retry        = 0;
    attr->alt_port_num     = 0;
    attr->alt_timeout      = 0;

    if (init_attr) {
        memset(init_attr, 0, sizeof(*init_attr));
        init_attr->qp_type = qp->qp_type;
        init_attr->send_cq = qp->send_cq;
        init_attr->recv_cq = qp->recv_cq;
        init_attr->cap     = attr->cap;
    }

    ODL_TRACE_EXIT_VAL(0);
}
