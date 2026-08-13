/*
 * OdinLink — Verbs: Memory Regions (Pinning RAM and GPU Memory for DMA)
 *
 * ibv_reg_mr:        Tell the kernel "this chunk of host RAM is safe to
 *                    DMA from/to." Pins the pages so they don't get
 *                    swapped out while the transfer is in flight.
 *
 * ibv_reg_dmabuf_mr: Same thing, but for GPU memory. Passes a DMA-buf
 *                    file descriptor to the kernel driver so it can DMA
 *                    directly from/to GPU VRAM — zero copies. This is
 *                    the key to fast GPU collectives (NCCL).
 */

#include "odl_tb5_verbs.h"
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

/* Allocate a non-zero, process-unique MR key (V7).  Keys are no longer derived
 * from the MR pointer, so a peer learning an rkey cannot infer our address
 * space layout, and distinct MRs cannot collide on low pointer bits.  With at
 * most ODL_VERBS_MAX_MRS live MRs and a 32-bit counter, wrap is impossible in
 * practice, so no in-use collision check is needed. */
static uint32_t odl_alloc_mr_handle(struct odl_verbs_context *ctx)
{
	uint32_t h;

	pthread_mutex_lock(&ctx->mr_lock);
	h = ctx->mr_seq + 1;
	if (h == 0)
		h = 1;
	ctx->mr_seq = h;
	pthread_mutex_unlock(&ctx->mr_lock);
	return h;
}

struct ibv_mr *odl_reg_mr(struct ibv_pd *pd, void *addr,
                           size_t length, uint64_t hca_va,
                           int access)
{
    ODL_TRACE_ENTRY();
    struct odl_verbs_context *ctx = odl_ctx_from_ibv(pd->context);

    struct odl_verbs_mr *mr = calloc(1, sizeof(*mr));
    if (!mr) { errno = ENOMEM; return NULL; }

    mr->base.addr    = addr;
    mr->base.length  = length;
    mr->base.handle  = 0;
    /* Unique, non-zero key (V7): monotonic handle, not a pointer.  RDMA
     * WRITE/READ carry this rkey on the wire and the responder reverse-maps it
     * (odl_find_mr_by_rkey) to find where remote_addr lands. */
    mr->base.lkey    = odl_alloc_mr_handle(ctx);
    mr->base.rkey    = mr->base.lkey;
    mr->base.context = pd->context;
    mr->mr_type      = 0; /* host */
    mr->access_flags = access;
    mr->host_addr    = addr;
    mr->host_length  = length;
    mr->dmabuf_fd    = -1;
    mr->iova         = hca_va ? hca_va : (uint64_t)(uintptr_t)addr;

    pthread_mutex_lock(&ctx->mr_lock);
    if (ctx->nmrs >= ODL_VERBS_MAX_MRS) {
        pthread_mutex_unlock(&ctx->mr_lock);
        free(mr);
        errno = ENOMEM;
        return NULL;
    }
    ctx->mrs[ctx->nmrs++] = mr;
    pthread_mutex_unlock(&ctx->mr_lock);

    odl_loginfo("reg_mr: addr=%p len=%zu lkey=%06x rkey=%06x",
                 addr, length, mr->base.lkey, mr->base.rkey);
    ODL_TRACE_EXIT();
    return &mr->base;
}

struct ibv_mr *odl_reg_dmabuf_mr(struct ibv_pd *pd, uint64_t offset,
                                  size_t length, uint64_t iova,
                                  int fd, int access)
{
    ODL_TRACE_ENTRY();
    struct odl_verbs_context *ctx = odl_ctx_from_ibv(pd->context);

    struct odl_verbs_mr *mr = calloc(1, sizeof(*mr));
    if (!mr) { errno = ENOMEM; return NULL; }

    /* Duplicate the dmabuf fd so we own it */
    mr->dmabuf_fd = fcntl(fd, F_DUPFD_CLOEXEC, 3);
    if (mr->dmabuf_fd < 0) {
        odl_logerr("fcntl(F_DUPFD_CLOEXEC) failed: %s", strerror(errno));
        free(mr);
        return NULL;
    }

    mr->base.addr        = NULL;
    mr->base.length      = length;
    mr->base.handle      = 0;
    mr->base.context     = pd->context;
    mr->mr_type          = 1; /* dmabuf */
    mr->access_flags     = access;
    mr->dmabuf_offset    = offset;
    mr->iova             = iova;
    mr->host_addr        = NULL;
    mr->host_length      = 0;

    /* Unique, non-zero key (V7): monotonic handle, not a pointer. */
    mr->base.lkey = odl_alloc_mr_handle(ctx);
    mr->base.rkey = mr->base.lkey;

    pthread_mutex_lock(&ctx->mr_lock);
    if (ctx->nmrs >= ODL_VERBS_MAX_MRS) {
        pthread_mutex_unlock(&ctx->mr_lock);
        close(mr->dmabuf_fd);
        free(mr);
        errno = ENOMEM;
        return NULL;
    }
    ctx->mrs[ctx->nmrs++] = mr;
    pthread_mutex_unlock(&ctx->mr_lock);

    odl_loginfo("reg_dmabuf_mr: fd=%d offset=%llu len=%zu iova=%llx "
                 "lkey=%06x",
                 fd, (unsigned long long)offset, length,
                 (unsigned long long)iova, mr->base.lkey);
    ODL_TRACE_EXIT();
    return &mr->base;
}

/* Find a local MR by the rkey a remote peer advertised.  Both host and dmabuf
 * MRs set base.rkey == base.lkey == (uint32_t)(uintptr_t)mr, so this doubles as
 * the send-side lkey lookup used to pick host vs. zero-copy dmabuf. */
struct odl_verbs_mr *odl_find_mr_by_rkey(struct odl_verbs_context *ctx,
                                         uint32_t rkey)
{
    if (!rkey) return NULL;
    pthread_mutex_lock(&ctx->mr_lock);
    for (int i = 0; i < ctx->nmrs; i++) {
        struct odl_verbs_mr *mr = ctx->mrs[i];
        if (mr && (mr->base.rkey == rkey || mr->base.lkey == rkey)) {
            pthread_mutex_unlock(&ctx->mr_lock);
            return mr;
        }
    }
    pthread_mutex_unlock(&ctx->mr_lock);
    return NULL;
}

int odl_dereg_mr(struct ibv_mr *mr)
{
    ODL_TRACE_ENTRY();
    if (!mr) return -EINVAL;

    struct odl_verbs_mr *omr = odl_mr_from_ibv(mr);
    struct odl_verbs_context *ctx = odl_ctx_from_ibv(mr->context);

    pthread_mutex_lock(&ctx->mr_lock);
    for (int i = 0; i < ctx->nmrs; i++) {
        if (ctx->mrs[i] == omr) {
            ctx->mrs[i] = ctx->mrs[--ctx->nmrs];
            break;
        }
    }
    pthread_mutex_unlock(&ctx->mr_lock);

    if (omr->dmabuf_fd >= 0) {
        close(omr->dmabuf_fd);
        odl_loginfo("dereg_mr: dmabuf fd=%d closed", omr->dmabuf_fd);
    }

    free(omr);
    ODL_TRACE_EXIT_VAL(0);
}
