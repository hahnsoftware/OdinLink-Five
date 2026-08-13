/*
 * OdinLink — Verbs: The Wiring That Connects ibv_* Calls to Our Code
 *
 * When an app calls ibv_query_device, ibv_post_send, ibv_poll_cq, etc.,
 * libibverbs looks up the function pointer in a dispatch table. This
 * file sets up that table so every standard verbs call routes to our
 * OdinLink implementation.
 *
 * For ibv_query_device and ibv_query_port, modern rdma-core uses an
 * internal struct (verbs_context) instead of the old ops table, so we
 * intercept those symbols directly via symbol interposition. The rest
 * (poll_cq, post_send, post_recv, etc.) go through the ops fields we
 * populate here.
 */

#include "odl_tb5_verbs.h"
#define _GNU_SOURCE
#include <dlfcn.h>

/* The struct _compat_ibv_port_attr definition from rdma-core:
 * Same layout as ibv_port_attr but used for the legacy dispatch path. */
struct _compat_ibv_port_attr {
    enum ibv_port_state state;
    enum ibv_mtu max_mtu;
    enum ibv_mtu active_mtu;
    int gid_tbl_len;
    uint32_t port_cap_flags;
    uint32_t max_msg_sz;
    uint32_t bad_pkey_cntr;
    uint32_t qkey_viol_cntr;
    uint16_t pkey_tbl_len;
    uint16_t lid;
    uint16_t sm_lid;
    uint8_t lmc;
    uint8_t max_vl_num;
    uint8_t sm_sl;
    uint8_t subnet_timeout;
    uint8_t init_type_reply;
    uint8_t active_width;
    uint8_t active_speed;
    uint8_t phys_state;
    uint8_t link_layer;
    uint8_t flags;
};

/* ── Legacy compat: ibv_query_device (via _compat_query_device) ─────── */

static int odl_compat_query_device(struct ibv_context *context,
                                    struct ibv_device_attr *attr)
{
    ODL_TRACE_ENTRY();
    (void)context;

    memset(attr, 0, sizeof(*attr));
    attr->phys_port_cnt    = 1;
    attr->max_qp           = ODL_VERBS_MAX_QPS;
    attr->max_qp_wr        = ODL_VERBS_SQ_DEPTH;
    attr->max_sge          = 1;
    attr->max_cq           = ODL_VERBS_MAX_CQS;
    attr->max_cqe          = ODL_VERBS_COMP_CHANNEL_BACKLOG;
    attr->max_mr           = ODL_VERBS_MAX_MRS;
    attr->max_pd           = ODL_VERBS_MAX_PDS;
    attr->max_mr_size      = SIZE_MAX;
    /* One-sided ops are emulated over the stream transport (see qp.c).  Report
     * outstanding-read/atomic depth so ib_read_bw and RCCL negotiate a
     * non-zero max_rd_atomic instead of refusing RDMA READ. */
    attr->max_qp_rd_atom      = 16;
    attr->max_qp_init_rd_atom = 16;
    attr->max_res_rd_atom     = ODL_VERBS_MAX_QPS * 16;
    attr->device_cap_flags    = IBV_DEVICE_RC_RNR_NAK_GEN;

    ODL_TRACE_EXIT_VAL(0);
}

/* ── Legacy compat: ibv_query_port (via _compat_query_port) ─────────── */

static int odl_compat_query_port(struct ibv_context *context,
                                  uint8_t port_num,
                                  struct _compat_ibv_port_attr *attr)
{
    ODL_TRACE_ENTRY();
    struct odl_verbs_context *ctx = odl_ctx_from_ibv(context);

    if (port_num != 1) return -EINVAL;

    memset(attr, 0, sizeof(*attr));

    struct odl_tb5_peer_info peer;
    bool connected = (odl_tb5_get_peer(ctx->handle, &peer) == 0 &&
                      peer.state >= ODL_TB5_STATE_CONNECTED);

    attr->max_mtu       = IBV_MTU_4096;
    attr->active_mtu    = IBV_MTU_4096;
    attr->gid_tbl_len   = 1;
    attr->port_cap_flags = IBV_PORT_CM_SUP;
    attr->max_msg_sz    = 1 << 20;
    attr->bad_pkey_cntr  = 0;
    attr->qkey_viol_cntr = 0;
    attr->pkey_tbl_len  = 0;
    attr->lid           = 0;
    attr->sm_lid        = 0;
    attr->lmc           = 0;
    attr->sm_sl         = 0;
    attr->subnet_timeout = 0;
    attr->init_type_reply = 0;
    attr->max_vl_num    = 1;
    attr->active_width = IBV_WIDTH_2X;
    attr->active_speed = IBV_SPEED_QDR;
    attr->link_layer   = IBV_LINK_LAYER_INFINIBAND;

    if (connected) {
        attr->state        = IBV_PORT_ACTIVE;
        attr->phys_state   = 5;
    } else {
        attr->state        = IBV_PORT_DOWN;
        attr->phys_state   = 3;
    }

    ODL_TRACE_EXIT_VAL(0);
}

/* ── Connpletion channel and async event stubs ──────────────────────── */

static struct ibv_comp_channel *odl_compat_create_comp_channel(
    struct ibv_context *context)
{
    ODL_TRACE_ENTRY();
    struct odl_verbs_context *ctx = odl_ctx_from_ibv(context);
    (void)ctx;
    errno = ENOSYS;
    return NULL;
}

static int odl_compat_destroy_comp_channel(
    struct ibv_comp_channel *channel)
{
    ODL_TRACE_ENTRY();
    (void)channel;
    return 0;
}

/* ── Symbol Interposition ──────────────────────────────────────────────
 *
 * Modern rdma-core (≥ 50) dispatches many ibv_* functions through the
 * internal verbs_context struct rather than ibv_context_ops. Since our
 * standalone library creates ibv_context without a verbs_context wrapper,
 * we intercept these functions via symbol interposition.
 *
 * For manual ODL contexts: dispatch directly to our implementation.
 * For all other contexts: chain to the real libibverbs function.
 *
 * Note: some libibverbs functions are declared as static inline in verbs.h
 * (poll_cq, post_send, post_recv, req_notify_cq). These dispatch through
 * ibv_context_ops which we set up — no interposition needed. Others are
 * real libibverbs symbols that need interposition. Undef them below. */
#undef ibv_query_port
#undef ibv_reg_mr
#undef ibv_dereg_mr
#undef ibv_create_cq
#undef ibv_destroy_cq
#undef ibv_create_qp
#undef ibv_destroy_qp
#undef ibv_modify_qp
#undef ibv_query_qp
#undef ibv_alloc_pd
#undef ibv_dealloc_pd

/* Helper: check if an ibv_context or ibv_pd/ibv_cq/ibv_qp belongs to us */
static bool is_odl_ctx(struct ibv_context *ctx)
{
    return ctx && ctx->device && ctx->device->name &&
           strncmp(ctx->device->name, "odl_tb5_", 8) == 0;
}
static bool is_odl_pd(struct ibv_pd *pd)
{
    return pd && is_odl_ctx(pd->context);
}
static bool is_odl_cq(struct ibv_cq *cq)
{
    return cq && is_odl_ctx(cq->context);
}
static bool is_odl_qp(struct ibv_qp *qp)
{
    return qp && is_odl_ctx(qp->context);
}

/* Resolve a real libibverbs function via dlsym(RTLD_NEXT) */
/* Note: only use for functions that return int and have a matching signature.
 * The typeof() approach fails on ARM64 with some GCC versions. */
static void *resolve_verbs_func(const char *name)
{
    static void *(*dlsym_fn)(void *, const char *) = NULL;
    if (!dlsym_fn) dlsym_fn = dlsym;
    return dlsym_fn(RTLD_NEXT, name);
}

/* ── ibv_close_device ──────────────────────────────────────────────── */

int ibv_close_device(struct ibv_context *context)
{
    ODL_TRACE_ENTRY();
    if (is_odl_ctx(context))
        return odl_free_context(context);
    int (*real_fn)(struct ibv_context *) = resolve_verbs_func("ibv_close_device");
    return real_fn ? real_fn(context) : -ENOSYS;
}

/* ── ibv_query_device ───────────────────────────────────────────────── */

int ibv_query_device(struct ibv_context *context,
                      struct ibv_device_attr *device_attr)
{
    ODL_TRACE_ENTRY();
    if (is_odl_ctx(context))
        return odl_compat_query_device(context, device_attr);
    int (*real_fn)(struct ibv_context *, struct ibv_device_attr *) = resolve_verbs_func("ibv_query_device");
    return real_fn ? real_fn(context, device_attr) : -ENOSYS;
}

/* ── ibv_query_port ─────────────────────────────────────────────────── */

int ibv_query_port(struct ibv_context *context, uint8_t port_num,
                    struct _compat_ibv_port_attr *port_attr)
{
    ODL_TRACE_ENTRY();
    if (is_odl_ctx(context))
        return odl_compat_query_port(context, port_num, port_attr);
    int (*real_fn)(struct ibv_context *, uint8_t, struct _compat_ibv_port_attr *) = resolve_verbs_func("ibv_query_port");
    return real_fn ? real_fn(context, port_num, port_attr) : -ENOSYS;
}

/* ── ibv_alloc_pd / ibv_dealloc_pd ──────────────────────────────────── */

struct ibv_pd *ibv_alloc_pd(struct ibv_context *context)
{
    ODL_TRACE_ENTRY();
    if (is_odl_ctx(context))
        return odl_alloc_pd(context);
    static struct ibv_pd *(*real_fn)(struct ibv_context *);
    if (!real_fn) { real_fn = dlsym(RTLD_NEXT, "ibv_alloc_pd"); }
    return real_fn ? real_fn(context) : NULL;
}

int ibv_dealloc_pd(struct ibv_pd *pd)
{
    ODL_TRACE_ENTRY();
    if (is_odl_pd(pd))
        return odl_dealloc_pd(pd);
    int (*real_fn)(struct ibv_pd *) = resolve_verbs_func("ibv_dealloc_pd");
    return real_fn ? real_fn(pd) : -ENOSYS;
}

/* ── ibv_reg_mr / ibv_dereg_mr ──────────────────────────────────────── */

struct ibv_mr *ibv_reg_mr(struct ibv_pd *pd, void *addr, size_t length,
                           int access)
{
    ODL_TRACE_ENTRY();
    if (is_odl_pd(pd))
        return odl_reg_mr(pd, addr, length, 0, access);
    static struct ibv_mr *(*real_fn)(struct ibv_pd *, void *, size_t, int);
    if (!real_fn) { real_fn = dlsym(RTLD_NEXT, "ibv_reg_mr"); }
    return real_fn ? real_fn(pd, addr, length, access) : NULL;
}

int ibv_dereg_mr(struct ibv_mr *mr)
{
    ODL_TRACE_ENTRY();
    if (mr && mr->context && is_odl_ctx(mr->context))
        return odl_dereg_mr(mr);
    int (*real_fn)(struct ibv_mr *) = resolve_verbs_func("ibv_dereg_mr");
    return real_fn ? real_fn(mr) : -ENOSYS;
}

/* Modern <infiniband/verbs.h> expands ibv_reg_mr(...) to a call to
 * ibv_reg_mr_iova2(), so a caller compiled against current rdma-core (e.g.
 * perftest) never reaches our ibv_reg_mr symbol above — it lands here.
 * Route odl PDs to the same odl_reg_mr; forward everything else. */
struct ibv_mr *ibv_reg_mr_iova2(struct ibv_pd *pd, void *addr, size_t length,
                                uint64_t iova, unsigned int access)
{
    ODL_TRACE_ENTRY();
    if (is_odl_pd(pd))
        return odl_reg_mr(pd, addr, length, iova, (int)access);
    static struct ibv_mr *(*real_fn)(struct ibv_pd *, void *, size_t,
                                     uint64_t, unsigned int);
    if (!real_fn) { real_fn = dlsym(RTLD_NEXT, "ibv_reg_mr_iova2"); }
    return real_fn ? real_fn(pd, addr, length, iova, access) : NULL;
}

/* ── ibv_reg_dmabuf_mr ──────────────────────────────────────────────────
 * The zero-copy GPU path (RCCL, and any app registering GPU/dma_heap memory)
 * registers through ibv_reg_dmabuf_mr, a real libibverbs symbol that would
 * otherwise dispatch through the verbs_context our standalone lib doesn't wrap
 * (garbage/EOPNOTSUPP on our fake context — same reason ibv_reg_mr_iova2 is
 * interposed).  Route odl PDs to odl_reg_dmabuf_mr (mr_type=1, zero-copy send
 * AND recv); forward everything else. */
struct ibv_mr *ibv_reg_dmabuf_mr(struct ibv_pd *pd, uint64_t offset,
                                 size_t length, uint64_t iova,
                                 int fd, int access)
{
    ODL_TRACE_ENTRY();
    if (is_odl_pd(pd))
        return odl_reg_dmabuf_mr(pd, offset, length, iova, fd, access);
    static struct ibv_mr *(*real_fn)(struct ibv_pd *, uint64_t, size_t,
                                     uint64_t, int, int);
    if (!real_fn) { real_fn = dlsym(RTLD_NEXT, "ibv_reg_dmabuf_mr"); }
    return real_fn ? real_fn(pd, offset, length, iova, fd, access) : NULL;
}

/* ── ibv_query_gid ──────────────────────────────────────────────────────
 * perftest queries the local GID to build its connection address vector.
 * We have no real GID table; return an all-zero GID (consistent with the
 * zero LID/GID query_port reports) so the standard tool can proceed instead
 * of falling through to real libibverbs on our synthetic context. */
int ibv_query_gid(struct ibv_context *context, uint8_t port_num,
                  int index, union ibv_gid *gid)
{
    ODL_TRACE_ENTRY();
    if (is_odl_ctx(context)) {
        if (gid)
            memset(gid, 0, sizeof(*gid));
        return 0;
    }
    int (*real_fn)(struct ibv_context *, uint8_t, int, union ibv_gid *) =
        resolve_verbs_func("ibv_query_gid");
    return real_fn ? real_fn(context, port_num, index, gid) : -ENOSYS;
}

/* Current rdma-core users prefer ibv_query_gid_ex() because it returns the
 * address and its type in one call.  A synthetic OdinLink context has no
 * rdma-core-internal GID table, so letting this fall through to libibverbs
 * dereferences state that does not exist. */
int _ibv_query_gid_ex(struct ibv_context *context, uint32_t port_num,
                      uint32_t gid_index, struct ibv_gid_entry *entry,
                      uint32_t flags, size_t entry_size)
{
    ODL_TRACE_ENTRY();
    if (is_odl_ctx(context)) {
        if (!entry || port_num != 1 || gid_index != 0 || flags != 0 ||
            entry_size < sizeof(*entry))
            return EINVAL;
        memset(entry, 0, sizeof(*entry));
        entry->gid_type = IBV_GID_TYPE_IB;
        return 0;
    }
    int (*real_fn)(struct ibv_context *, uint32_t, uint32_t,
                   struct ibv_gid_entry *, uint32_t, size_t) =
        resolve_verbs_func("_ibv_query_gid_ex");
    return real_fn ? real_fn(context, port_num, gid_index, entry, flags,
                             entry_size)
                   : ENOSYS;
}
/* ── ibv_create_cq / ibv_destroy_cq ─────────────────────────────────── */

struct ibv_cq *ibv_create_cq(struct ibv_context *context, int cqe,
                              void *cq_context,
                              struct ibv_comp_channel *channel,
                              int comp_vector)
{
    ODL_TRACE_ENTRY();
    if (is_odl_ctx(context))
        return odl_create_cq(context, cqe, channel, comp_vector);
    static struct ibv_cq *(*real_fn)(struct ibv_context *, int, void *,
                                      struct ibv_comp_channel *, int);
    if (!real_fn) { real_fn = dlsym(RTLD_NEXT, "ibv_create_cq"); }
    return real_fn ? real_fn(context, cqe, cq_context, channel, comp_vector) : NULL;
}

int ibv_destroy_cq(struct ibv_cq *cq)
{
    ODL_TRACE_ENTRY();
    if (is_odl_cq(cq))
        return odl_destroy_cq(cq);
    int (*real_fn)(struct ibv_cq *) = resolve_verbs_func("ibv_destroy_cq");
    return real_fn ? real_fn(cq) : -ENOSYS;
}

/* ── ibv_create_qp / ibv_destroy_qp / ibv_modify_qp / ibv_query_qp ──── */

struct ibv_qp *ibv_create_qp(struct ibv_pd *pd,
                              struct ibv_qp_init_attr *attr)
{
    ODL_TRACE_ENTRY();
    if (is_odl_pd(pd))
        return odl_create_qp(pd, attr);
    static struct ibv_qp *(*real_fn)(struct ibv_pd *, struct ibv_qp_init_attr *);
    if (!real_fn) { real_fn = dlsym(RTLD_NEXT, "ibv_create_qp"); }
    return real_fn ? real_fn(pd, attr) : NULL;
}

int ibv_destroy_qp(struct ibv_qp *qp)
{
    ODL_TRACE_ENTRY();
    if (is_odl_qp(qp))
        return odl_destroy_qp(qp);
    int (*real_fn)(struct ibv_qp *) = resolve_verbs_func("ibv_destroy_qp");
    return real_fn ? real_fn(qp) : -ENOSYS;
}

int ibv_modify_qp(struct ibv_qp *qp, struct ibv_qp_attr *attr,
                   int attr_mask)
{
    ODL_TRACE_ENTRY();
    if (is_odl_qp(qp))
        return odl_modify_qp(qp, attr, attr_mask);
    int (*real_fn)(struct ibv_qp *, struct ibv_qp_attr *, int) = resolve_verbs_func("ibv_modify_qp");
    return real_fn ? real_fn(qp, attr, attr_mask) : -ENOSYS;
}

int ibv_query_qp(struct ibv_qp *qp, struct ibv_qp_attr *attr,
                  int attr_mask, struct ibv_qp_init_attr *init_attr)
{
    ODL_TRACE_ENTRY();
    if (is_odl_qp(qp))
        return odl_query_qp(qp, attr, attr_mask, init_attr);
    int (*real_fn)(struct ibv_qp *, struct ibv_qp_attr *, int, struct ibv_qp_init_attr *) = resolve_verbs_func("ibv_query_qp");
    return real_fn ? real_fn(qp, attr, attr_mask, init_attr) : -ENOSYS;
}

/* RCCL resolves these calls directly from its dlopen() handle instead of
 * using the process-wide symbol table. Export thin versions here so that
 * handle contains the complete verbs surface RCCL checks at startup. */
#undef ibv_poll_cq
#undef ibv_post_send
#undef ibv_post_recv

int odl_ibv_poll_cq_export(struct ibv_cq *cq, int num_entries,
                           struct ibv_wc *wc) __asm__("ibv_poll_cq");
int odl_ibv_post_send_export(struct ibv_qp *qp, struct ibv_send_wr *wr,
                             struct ibv_send_wr **bad_wr)
    __asm__("ibv_post_send");
int odl_ibv_post_recv_export(struct ibv_qp *qp, struct ibv_recv_wr *wr,
                             struct ibv_recv_wr **bad_wr)
    __asm__("ibv_post_recv");

const char *ibv_get_device_name(struct ibv_device *device)
{
    return device ? device->name : NULL;
}

int odl_ibv_poll_cq_export(struct ibv_cq *cq, int num_entries, struct ibv_wc *wc)
{
    if (is_odl_cq(cq))
        return odl_poll_cq(cq, num_entries, wc);
    int (*real_fn)(struct ibv_cq *, int, struct ibv_wc *) =
        resolve_verbs_func("ibv_poll_cq");
    return real_fn ? real_fn(cq, num_entries, wc) : -ENOSYS;
}

int odl_ibv_post_send_export(struct ibv_qp *qp, struct ibv_send_wr *wr,
                  struct ibv_send_wr **bad_wr)
{
    if (is_odl_qp(qp))
        return odl_post_send(qp, wr, bad_wr);
    int (*real_fn)(struct ibv_qp *, struct ibv_send_wr *,
                   struct ibv_send_wr **) =
        resolve_verbs_func("ibv_post_send");
    return real_fn ? real_fn(qp, wr, bad_wr) : ENOSYS;
}

int odl_ibv_post_recv_export(struct ibv_qp *qp, struct ibv_recv_wr *wr,
                  struct ibv_recv_wr **bad_wr)
{
    if (is_odl_qp(qp))
        return odl_post_recv(qp, wr, bad_wr);
    int (*real_fn)(struct ibv_qp *, struct ibv_recv_wr *,
                   struct ibv_recv_wr **) =
        resolve_verbs_func("ibv_post_recv");
    return real_fn ? real_fn(qp, wr, bad_wr) : ENOSYS;
}

int ibv_get_async_event(struct ibv_context *context,
                        struct ibv_async_event *event)
{
    if (is_odl_ctx(context)) {
        errno = EOPNOTSUPP;
        return -1;
    }
    int (*real_fn)(struct ibv_context *, struct ibv_async_event *) =
        resolve_verbs_func("ibv_get_async_event");
    return real_fn ? real_fn(context, event) : -1;
}

void ibv_ack_async_event(struct ibv_async_event *event)
{
    void (*real_fn)(struct ibv_async_event *) =
        resolve_verbs_func("ibv_ack_async_event");
    if (real_fn)
        real_fn(event);
}

int ibv_query_ece(struct ibv_qp *qp, struct ibv_ece *ece)
{
    if (is_odl_qp(qp))
        return EOPNOTSUPP;
    int (*real_fn)(struct ibv_qp *, struct ibv_ece *) =
        resolve_verbs_func("ibv_query_ece");
    return real_fn ? real_fn(qp, ece) : EOPNOTSUPP;
}

int ibv_set_ece(struct ibv_qp *qp, struct ibv_ece *ece)
{
    if (is_odl_qp(qp))
        return EOPNOTSUPP;
    int (*real_fn)(struct ibv_qp *, struct ibv_ece *) =
        resolve_verbs_func("ibv_set_ece");
    return real_fn ? real_fn(qp, ece) : EOPNOTSUPP;
}

int ibv_fork_init(void)
{
    int (*real_fn)(void) = resolve_verbs_func("ibv_fork_init");
    return real_fn ? real_fn() : ENOSYS;
}

const char *ibv_event_type_str(enum ibv_event_type event)
{
    const char *(*real_fn)(enum ibv_event_type) =
        resolve_verbs_func("ibv_event_type_str");
    return real_fn ? real_fn(event) : "unknown";
}

/* ── Context Ops Table ─────────────────────────────────────────────── */

void odl_init_context_ops(struct ibv_context *ctx)
{
    /* Legacy _compat_ entries */
    ctx->ops._compat_query_device = odl_compat_query_device;
    ctx->ops._compat_query_port   = odl_compat_query_port;
    ctx->ops._compat_alloc_pd     = (void*)odl_alloc_pd;
    ctx->ops._compat_dealloc_pd   = (void*)odl_dealloc_pd;
    ctx->ops._compat_reg_mr       = (void*)odl_reg_mr;
    ctx->ops._compat_dereg_mr     = (void*)odl_dereg_mr;
    ctx->ops._compat_create_cq    = (void*)odl_create_cq;
    ctx->ops._compat_destroy_cq   = (void*)odl_destroy_cq;
    ctx->ops._compat_resize_cq    = NULL;
    ctx->ops._compat_cq_event     = (void*)odl_cq_event;
    ctx->ops._compat_create_qp    = (void*)odl_create_qp;
    ctx->ops._compat_destroy_qp   = (void*)odl_destroy_qp;
    ctx->ops._compat_modify_qp    = (void*)odl_modify_qp;
    ctx->ops._compat_query_qp     = (void*)odl_query_qp;
    ctx->ops._compat_create_srq   = NULL;
    ctx->ops._compat_modify_srq   = NULL;
    ctx->ops._compat_query_srq    = NULL;
    ctx->ops._compat_destroy_srq  = NULL;
    ctx->ops._compat_create_ah    = NULL;
    ctx->ops._compat_destroy_ah   = NULL;
    ctx->ops._compat_attach_mcast  = NULL;
    ctx->ops._compat_detach_mcast  = NULL;
    ctx->ops._compat_async_event  = NULL;
    ctx->ops._compat_rereg_mr     = NULL;

    /* Non-compat entries (used by poll_cq, post_send, etc.) */
    ctx->ops.poll_cq         = odl_poll_cq;
    ctx->ops.req_notify_cq   = odl_req_notify_cq;
    ctx->ops.post_send       = odl_post_send;
    ctx->ops.post_recv       = odl_post_recv;
    ctx->ops.post_srq_recv   = NULL;
    ctx->ops.alloc_mw        = NULL;
    ctx->ops.dealloc_mw      = NULL;
    ctx->ops.bind_mw         = NULL;
}
