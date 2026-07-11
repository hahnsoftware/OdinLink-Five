/*
 * OdinLink — Verbs: Queue Pairs (The Actual Data Path)
 *
 * A QP (Queue Pair) is just a pair of send/receive queues — like a
 * pipe between two machines. This file maps an RDMA QP to an OdinLink
 * stream.
 *
 * How async I/O works here:
 * 1. ibv_post_send → enqueue the work request → return immediately
 * 2. A background worker thread polls the device fd (EPOLLOUT)
 * 3. When the hardware is ready, it dequeues a WR and calls the
 *    kernel's stream_send ioctl (which is O_NONBLOCK)
 * 4. If the kernel says EAGAIN (busy), re-enqueue and poll again
 * 5. On success, post a Work Completion to the CQ and wake the app
 *    via eventfd
 *
 * This gives true async I/O: the calling thread never blocks even
 * though the kernel path is synchronous under the hood.
 */

#include "odl_tb5_verbs.h"
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <poll.h>
#include <sched.h>

/* ── Worker Thread ──────────────────────────────────────────────────── */

static int odl_worker_poll_fd(struct odl_verbs_qp *qp, int timeout_ms)
{
    struct odl_verbs_context *ctx = qp->ctx;
    int dev_fd = ctx->base.cmd_fd;
    if (dev_fd < 0) return -EBADF;

    struct pollfd pfd = {
        .fd     = dev_fd,
        .events = POLLOUT,
    };

    int ret = poll(&pfd, 1, timeout_ms);
    if (ret < 0) return -errno;
    if (ret == 0) return -ETIMEDOUT;
    return (pfd.revents & POLLOUT) ? 0 : -EAGAIN;
}

/* Look up a dmabuf fd by lkey. Returns -1 if not a dmabuf MR. */
static int odl_lookup_dmabuf(struct odl_verbs_context *ctx, uint32_t lkey)
{
    if (!lkey) return -1;
    pthread_mutex_lock(&ctx->mr_lock);
    for (int i = 0; i < ctx->nmrs; i++) {
        struct odl_verbs_mr *mr = ctx->mrs[i];
        if (mr && mr->base.lkey == lkey && mr->mr_type == 1) {
            int fd = mr->dmabuf_fd;
            pthread_mutex_unlock(&ctx->mr_lock);
            return fd;
        }
    }
    pthread_mutex_unlock(&ctx->mr_lock);
    return -1;
}

/* Drain at most one posted receive.  Returns true if a completion was posted.
 *
 * Runs on the dedicated recv worker (never the send worker), so it is free to
 * block: a dmabuf MR takes the zero-copy path (odl_tb5_stream_recv_dmabuf,
 * which blocks in the kernel until the DMA completes), mirroring the send-side
 * odl_lookup_dmabuf dispatch.  Host MRs keep the non-blocking stream_recv with
 * EAGAIN re-post, so an app that posts a receive before data arrives (perftest)
 * still works. */
static bool odl_qp_drain_recv(struct odl_verbs_qp *qp)
{
    struct odl_verbs_context *ctx = qp->ctx;

    pthread_mutex_lock(&qp->rq_lock);
    if (qp->rq_count == 0) {
        pthread_mutex_unlock(&qp->rq_lock);
        return false;
    }
    struct ibv_recv_wr *rwr = qp->rq[qp->rq_head];
    qp->rq_head = (qp->rq_head + 1) % ODL_VERBS_RQ_DEPTH;
    qp->rq_count--;
    pthread_mutex_unlock(&qp->rq_lock);

    uint8_t src_id = 0;
    uint32_t actual = 0;
    int ret = 0;
    struct ibv_sge *sge = (rwr->num_sge > 0) ? &rwr->sg_list[0] : NULL;

    if (sge) {
        int dmabuf_fd = odl_lookup_dmabuf(ctx, sge->lkey);

        if (dmabuf_fd >= 0) {
            /* Zero-copy GPU RX: DMA straight into the dmabuf.  Blocking —
             * safe here because sends run on the other worker.  Mirrors the
             * send side's offset 0 / full-length convention. */
            ret = odl_tb5_stream_recv_dmabuf(ctx->handle, qp->stream_id,
                                             dmabuf_fd, 0, sge->length);
            actual = (ret == 0) ? sge->length : 0;
            src_id = qp->dest_stream_id;
            if (ret != 0)
                odl_logverbose("RX dmabuf stream=%u ret=%d len=%u",
                            qp->stream_id, ret, sge->length);
        } else {
            ret = odl_tb5_stream_recv(ctx->handle, qp->stream_id,
                                      (void *)(uintptr_t)sge->addr,
                                      sge->length, &src_id, &actual);
            if (ret != -EAGAIN)
                odl_logverbose("RX drain stream=%u ret=%d actual=%u src=%u",
                            qp->stream_id, ret, actual, src_id);
            if (ret == -EAGAIN) {
                /* Data not here yet — re-post at the head so ordering is
                 * preserved and retry on a later tick. */
                pthread_mutex_lock(&qp->rq_lock);
                qp->rq_head = (qp->rq_head + ODL_VERBS_RQ_DEPTH - 1)
                              % ODL_VERBS_RQ_DEPTH;
                qp->rq[qp->rq_head] = rwr;
                qp->rq_count++;
                pthread_mutex_unlock(&qp->rq_lock);
                return false;
            }
        }
    }

    struct ibv_wc wc;
    memset(&wc, 0, sizeof(wc));
    wc.wr_id    = rwr->wr_id;
    wc.qp_num   = qp->base.qp_num;
    wc.src_qp   = src_id;
    wc.opcode   = IBV_WC_RECV;
    wc.status   = (ret == 0) ? IBV_WC_SUCCESS : IBV_WC_GENERAL_ERR;
    wc.byte_len = (ret == 0) ? actual : 0;

    if (ret != 0)
        odl_logerr("recv failed: stream=%u ret=%d", qp->stream_id, ret);

    if (qp->recv_cq)
        odl_cq_post(qp->recv_cq, &wc);
    atomic_fetch_sub(&qp->pending_recvs, 1);
    return true;
}

static void *odl_qp_worker(void *arg)
{
    struct odl_verbs_qp *qp = arg;
    struct odl_verbs_context *ctx = qp->ctx;
    odl_tb5_t h = ctx->handle;

    while (qp->worker_running) {
        struct ibv_send_wr *wr = NULL;

        /* Dequeue one work request */
        pthread_mutex_lock(&qp->sq_lock);
        if (qp->sq_count > 0) {
            wr = qp->sq[qp->sq_head];
            qp->sq_head = (qp->sq_head + 1) % ODL_VERBS_SQ_DEPTH;
            qp->sq_count--;
        }
        pthread_mutex_unlock(&qp->sq_lock);

        if (!wr) {
            /* No send work.  Receives are drained on the dedicated recv
             * worker, so just yield the CPU and re-check the SQ.  Busy-poll
             * (RDMA polling-mode style): perftest posts the next send to the
             * SQ without signalling the device fd, so a blocking poll would
             * cap latency at its timeout. */
            sched_yield();
            continue;
        }

        /* stream_send is non-blocking (O_NONBLOCK fd) and returns -EAGAIN if
         * no TX frames are available; the EAGAIN path below re-queues.  Do
         * NOT gate on poll(POLLOUT) here: the driver only raises EPOLLOUT on
         * path-level tx.completed, which stream sends never bump, so polling
         * it stalls for the full timeout on every send. */

        /* Execute the send (non-blocking — fd is O_NONBLOCK) */
        int ret;
        struct ibv_wc wc;
        memset(&wc, 0, sizeof(wc));

        if (wr->num_sge > 0) {
            struct ibv_sge *sge = &wr->sg_list[0];
            int dmabuf_fd = odl_lookup_dmabuf(ctx, sge->lkey);

            if (dmabuf_fd >= 0) {
                ret = odl_tb5_stream_send_dmabuf(
                    h, qp->stream_id, qp->dest_stream_id,
                    dmabuf_fd, 0, sge->length);
            } else {
                void *data = (void *)(uintptr_t)sge->addr;
                ret = odl_tb5_stream_send(
                    h, qp->stream_id, qp->dest_stream_id,
                    data, sge->length);
            }

            if (ret != -EAGAIN)
                odl_logverbose("TX stream=%u dst=%u ret=%d len=%u",
                            qp->stream_id, qp->dest_stream_id, ret,
                            sge->length);
            if (ret == -EAGAIN) {
                /* Non-blocking send couldn't proceed — re-queue and retry */
                odl_logverbose("send EAGAIN stream=%u, re-queueing", qp->stream_id);
                pthread_mutex_lock(&qp->sq_lock);
                if (qp->sq_count < ODL_VERBS_SQ_DEPTH) {
                    qp->sq[qp->sq_tail] = wr;
                    qp->sq_tail = (qp->sq_tail + 1) % ODL_VERBS_SQ_DEPTH;
                    qp->sq_count++;
                }
                pthread_mutex_unlock(&qp->sq_lock);
                continue;
            }

            wc.status   = (ret == 0) ? IBV_WC_SUCCESS : IBV_WC_GENERAL_ERR;
            wc.byte_len = (ret == 0) ? sge->length : 0;
            wc.opcode   = IBV_WC_SEND;
            wc.qp_num   = qp->base.qp_num;

            if (ret != 0)
                odl_logerr("send failed stream=%u ret=%d", qp->stream_id, ret);
        } else {
            /* Zero-length send */
            wc.status   = IBV_WC_SUCCESS;
            wc.byte_len = 0;
            wc.opcode   = IBV_WC_SEND;
            wc.qp_num   = qp->base.qp_num;
        }

        /* Post completion to send CQ */
        if (qp->send_cq)
            odl_cq_post(qp->send_cq, &wc);

        atomic_fetch_sub(&qp->pending_sends, 1);
    }

    return NULL;
}

/* Dedicated recv worker: owns the RQ so a blocking dmabuf recv here can never
 * hold up a send (which runs on odl_qp_worker).  Busy-polls like the send side
 * for the host path; a dmabuf recv parks in the kernel until the DMA lands. */
static void *odl_qp_recv_worker(void *arg)
{
    struct odl_verbs_qp *qp = arg;

    while (qp->recv_worker_running) {
        if (!odl_qp_drain_recv(qp))
            sched_yield();
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

    /* Initialize SQ */
    pthread_mutex_init(&qp->sq_lock, NULL);
    qp->sq_head  = 0;
    qp->sq_tail  = 0;
    qp->sq_count = 0;

    /* Initialize RQ */
    pthread_mutex_init(&qp->rq_lock, NULL);
    qp->rq_head  = 0;
    qp->rq_tail  = 0;
    qp->rq_count = 0;

    atomic_init(&qp->pending_sends, 0);
    atomic_init(&qp->pending_recvs, 0);

    /* Track in context */
    pthread_mutex_lock(&ctx->qp_lock);
    if (ctx->nqps >= ODL_VERBS_MAX_QPS) {
        pthread_mutex_unlock(&ctx->qp_lock);
        odl_tb5_stream_close(ctx->handle, stream_id);
        pthread_mutex_destroy(&qp->sq_lock);
        pthread_mutex_destroy(&qp->rq_lock);
        free(qp);
        errno = ENOMEM;
        return NULL;
    }
    ctx->qps[ctx->nqps++] = qp;
    pthread_mutex_unlock(&ctx->qp_lock);

    /* Start async worker threads: one for sends, one for receives.  Splitting
     * them lets a blocking dmabuf recv run without stalling pending sends. */
    qp->worker_running = true;
    ret = pthread_create(&qp->worker, NULL, odl_qp_worker, qp);
    if (ret != 0) {
        odl_logerr("pthread_create (send) failed: %d", ret);
        qp->worker_running = false;
        odl_tb5_stream_close(ctx->handle, stream_id);
        pthread_mutex_destroy(&qp->sq_lock);
        pthread_mutex_destroy(&qp->rq_lock);
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

    /* Stop worker threads.  The recv worker only blocks in a dmabuf recv while
     * a receive is posted; a well-behaved app drains its receives before
     * destroy, so it is spinning on an empty RQ and joins promptly. */
    oqp->worker_running = false;
    oqp->recv_worker_running = false;
    pthread_join(oqp->worker, NULL);
    pthread_join(oqp->recv_worker, NULL);

    /* Close the OdinLink-Five stream */
    if (oqp->stream_id > 0)
        odl_tb5_stream_close(ctx->handle, oqp->stream_id);

    /* Remove from context */
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
    free(oqp);

    odl_loginfo("destroy_qp: stream=%u", oqp->stream_id);
    ODL_TRACE_EXIT_VAL(0);
}

/* ── Modify QP ─────────────────────────────────────────────────────── */

int odl_modify_qp(struct ibv_qp *qp, struct ibv_qp_attr *attr,
                   int attr_mask)
{
    ODL_TRACE_ENTRY();
    ODL_RETURN_EINVAL_IF(!qp, "null qp");

    /* Capture the peer's stream id from the RDMA connection handshake.
     * qp_num == stream_id on both ends, so dest_qp_num is the peer's stream;
     * the send worker uses it as the dst_id so data lands in the peer QP's
     * receive stream instead of a hardcoded 0. */
    if (attr_mask & IBV_QP_DEST_QPN) {
        struct odl_verbs_qp *oqp = odl_qp_from_ibv(qp);
        oqp->dest_stream_id = (uint8_t)attr->dest_qp_num;
        odl_loginfo("modify_qp: qp_num=%u dest_stream=%u",
                     qp->qp_num, oqp->dest_stream_id);
    }

    if (attr_mask & IBV_QP_STATE) {
        qp->state = attr->qp_state;
        odl_loginfo("modify_qp: qp_num=%u state=%d -> %d",
                     qp->qp_num, qp->state, attr->qp_state);

        /* On RTS transition, ensure peer is ready */
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
    *bad_wr = NULL;

    pthread_mutex_lock(&oqp->sq_lock);

    while (wr) {
        if (oqp->sq_count >= ODL_VERBS_SQ_DEPTH) {
            *bad_wr = wr;
            pthread_mutex_unlock(&oqp->sq_lock);
            odl_logerr("post_send: SQ full on QP %u", qp->qp_num);
            return -ENOMEM;
        }

        oqp->sq[oqp->sq_tail] = wr;
        oqp->sq_tail = (oqp->sq_tail + 1) % ODL_VERBS_SQ_DEPTH;
        oqp->sq_count++;
        atomic_fetch_add(&oqp->pending_sends, 1);

        wr = wr->next;
    }

    pthread_mutex_unlock(&oqp->sq_lock);
    return 0;
}

/* ── Post Recv (async: enqueue only, worker drains) ─────────────────────
 * RDMA semantics require post_recv to enqueue the receive buffer and return
 * immediately — the completion is delivered later via the CQ once data
 * arrives.  (The old implementation blocked here doing an inline stream_recv,
 * which fails for any app that posts receives before data exists, e.g.
 * perftest.)  We mirror the send path: enqueue into the RQ ring and let the
 * QP worker (odl_qp_drain_recv) consume arriving data and post IBV_WC_RECV. */
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
