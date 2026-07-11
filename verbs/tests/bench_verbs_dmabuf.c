/*
 * OdinLink — Verbs Zero-Copy DMA-buf Ping-Pong Benchmark
 *
 * Exercises the SYMMETRIC zero-copy verbs data path — the real GPU/RCCL
 * transport — end to end over two boxes:
 *
 *     ibv_post_send(dmabuf MR)  ->  odl_qp_worker      -> stream_send_dmabuf
 *     ibv_post_recv(dmabuf MR)  ->  odl_qp_recv_worker -> stream_recv_dmabuf
 *
 * Both sides register their buffers with ibv_reg_dmabuf_mr on a real dmabuf
 * fd from /dev/dma_heap/system (CPU-backed, no GPU needed — the DMA path
 * through the NHI is identical for an amdgpu/CUDA dmabuf).  perftest can only
 * reach the host-memory path (ibv_reg_mr), so this is the only tool that
 * proves the zero-copy recv wiring.
 *
 * Coordination is a self-synchronising ping-pong (echo), exactly like
 * tests/odl_tb5_bench_dmabuf.c: the client sends `size` bytes and waits for
 * the server to echo them back.  Separate send/recv buffers plus a per-size
 * pattern prove the bytes actually make the round trip — a no-op transport
 * leaves the recv buffer unchanged and shows up as INTEGRITY FAIL, never as a
 * bogus throughput number.  Start the server first, then the client.
 *
 * Build (on the box, links the standalone provider directly — no LD_PRELOAD):
 *   gcc -O2 -o bench_verbs_dmabuf bench_verbs_dmabuf.c \
 *       -I<repo>/verbs/src -L<build>/verbs -L<build>/lib \
 *       -lodl_tb5_verbs -libverbs -lpthread
 *   LD_LIBRARY_PATH=<build>/verbs:<build>/lib ./bench_verbs_dmabuf server
 *
 * Run:  server:  ./bench_verbs_dmabuf server
 *       client:  ./bench_verbs_dmabuf client
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/dma-heap.h>
#include <infiniband/verbs.h>

#include "odl_tb5_verbs_wrapper.h"

/* Default size sweep (bytes).  Kept <= 8M: the synchronous dmabuf path posts
 * one 4095-byte frame per chunk into a 4096-entry ring, so a single transfer
 * above ~16 MB returns -ENOSPC (RCCL chunks anyway). */
static size_t g_sizes[16] = { 65536, 262144, 1048576, 4194304, 8388608 };
static int    g_num_sizes = 5;
static int    g_iters     = 200;
static int    g_warmup    = 20;
static int    g_dev       = 0;

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

/* Allocate a real dmabuf from the system dma-heap (CPU-backed, no GPU). */
static int alloc_dmabuf(size_t size)
{
    int heap = open("/dev/dma_heap/system", O_RDWR | O_CLOEXEC);
    if (heap < 0) {
        fprintf(stderr,
            "FAIL: /dev/dma_heap/system: %s\n"
            "      (need CONFIG_DMABUF_HEAPS_SYSTEM)\n", strerror(errno));
        return -1;
    }
    struct dma_heap_allocation_data data = {
        .len      = size,
        .fd_flags = O_CLOEXEC | O_RDWR,
    };
    if (ioctl(heap, DMA_HEAP_IOCTL_ALLOC, &data) < 0) {
        fprintf(stderr, "FAIL: DMA_HEAP_IOCTL_ALLOC(%zu): %s\n",
                size, strerror(errno));
        close(heap);
        return -1;
    }
    close(heap);
    return (int)data.fd;
}

static int fill_fd(int fd, size_t size, unsigned char byte)
{
    void *m = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (m == MAP_FAILED)
        return -1;
    memset(m, byte, size);
    munmap(m, size);
    return 0;
}

/* Returns count of mismatching bytes (0 = clean round-trip). */
static size_t verify_fd(int fd, size_t size, unsigned char expect)
{
    void *m = mmap(NULL, size, PROT_READ, MAP_SHARED, fd, 0);
    if (m == MAP_FAILED)
        return size;
    const unsigned char *b = m;
    size_t bad = 0;
    for (size_t i = 0; i < size; i++)
        if (b[i] != expect)
            bad++;
    munmap(m, size);
    return bad;
}

static int cmp_u64(const void *a, const void *b)
{
    uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
    return (x > y) - (x < y);
}

static void fmt_thru(double gbps, char *buf, size_t n)
{
    snprintf(buf, n, "%.2f Gb/s (%.2f GB/s)", gbps, gbps / 8.0);
}

/* ── Verbs setup ────────────────────────────────────────────────────── */

struct vconn {
    struct ibv_context *ctx;
    struct ibv_pd      *pd;
    struct ibv_cq      *cq;
    struct ibv_qp      *qp;
};

static int conn_open(struct vconn *c)
{
    struct ibv_device *dev = odl_find_tb5_device(g_dev);
    if (!dev) {
        fprintf(stderr, "no OdinLink device %d (module loaded? peer up?)\n",
                g_dev);
        return -1;
    }
    c->ctx = ibv_open_device(dev);
    if (!c->ctx) { fprintf(stderr, "ibv_open_device failed\n"); return -1; }

    c->pd = ibv_alloc_pd(c->ctx);
    if (!c->pd) { fprintf(stderr, "ibv_alloc_pd failed\n"); return -1; }

    c->cq = ibv_create_cq(c->ctx, 64, NULL, NULL, 0);
    if (!c->cq) { fprintf(stderr, "ibv_create_cq failed\n"); return -1; }

    struct ibv_qp_init_attr qa;
    memset(&qa, 0, sizeof(qa));
    qa.send_cq = c->cq;
    qa.recv_cq = c->cq;
    qa.qp_type = IBV_QPT_RC;
    qa.cap.max_send_wr  = 16;
    qa.cap.max_recv_wr  = 16;
    qa.cap.max_send_sge = 1;
    qa.cap.max_recv_sge = 1;
    c->qp = ibv_create_qp(c->pd, &qa);
    if (!c->qp) { fprintf(stderr, "ibv_create_qp failed\n"); return -1; }

    /* INIT → RTR → RTS.  The dmabuf DMA path is raw paths[0] (stream routing
     * is ignored), so dest_qp_num is cosmetic here; we still drive the full
     * handshake so RTS runs the peer-ready wait like a real connection. */
    struct ibv_qp_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.qp_state        = IBV_QPS_INIT;
    attr.pkey_index      = 0;
    attr.port_num        = 1;
    attr.qp_access_flags = IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE;
    if (ibv_modify_qp(c->qp, &attr,
                      IBV_QP_STATE | IBV_QP_PKEY_INDEX |
                      IBV_QP_PORT | IBV_QP_ACCESS_FLAGS)) {
        fprintf(stderr, "modify_qp INIT failed\n"); return -1;
    }

    memset(&attr, 0, sizeof(attr));
    attr.qp_state           = IBV_QPS_RTR;
    attr.path_mtu           = IBV_MTU_4096;
    attr.dest_qp_num        = c->qp->qp_num;
    attr.rq_psn             = 0;
    attr.max_dest_rd_atomic = 1;
    attr.min_rnr_timer      = 12;
    if (ibv_modify_qp(c->qp, &attr,
                      IBV_QP_STATE | IBV_QP_AV | IBV_QP_PATH_MTU |
                      IBV_QP_DEST_QPN | IBV_QP_RQ_PSN |
                      IBV_QP_MAX_DEST_RD_ATOMIC | IBV_QP_MIN_RNR_TIMER)) {
        fprintf(stderr, "modify_qp RTR failed\n"); return -1;
    }

    memset(&attr, 0, sizeof(attr));
    attr.qp_state      = IBV_QPS_RTS;
    attr.timeout       = 14;
    attr.retry_cnt     = 7;
    attr.rnr_retry     = 7;
    attr.sq_psn        = 0;
    attr.max_rd_atomic = 1;
    if (ibv_modify_qp(c->qp, &attr,
                      IBV_QP_STATE | IBV_QP_TIMEOUT | IBV_QP_RETRY_CNT |
                      IBV_QP_RNR_RETRY | IBV_QP_SQ_PSN | IBV_QP_MAX_QP_RD_ATOMIC)) {
        fprintf(stderr, "modify_qp RTS failed\n"); return -1;
    }
    return 0;
}

static void conn_close(struct vconn *c)
{
    if (c->qp) ibv_destroy_qp(c->qp);
    if (c->cq) ibv_destroy_cq(c->cq);
    if (c->pd) ibv_dealloc_pd(c->pd);
    if (c->ctx) ibv_close_device(c->ctx);
}

/* Post a single WR from a dmabuf MR and wait (poll) for its completion. */
static int do_send(struct vconn *c, struct ibv_mr *mr, size_t len)
{
    struct ibv_sge sge = { .addr = 0, .length = (uint32_t)len, .lkey = mr->lkey };
    struct ibv_send_wr wr = {
        .wr_id = 1, .sg_list = &sge, .num_sge = 1,
        .opcode = IBV_WR_SEND, .send_flags = IBV_SEND_SIGNALED,
    };
    struct ibv_send_wr *bad = NULL;
    int ret = ibv_post_send(c->qp, &wr, &bad);
    if (ret) return -ret;

    struct ibv_wc wc;
    for (;;) {
        int n = ibv_poll_cq(c->cq, 1, &wc);
        if (n < 0) return -EIO;
        if (n == 0) continue;
        if (wc.status != IBV_WC_SUCCESS) return -EIO;
        return 0;
    }
}

static int do_recv(struct vconn *c, struct ibv_mr *mr, size_t len)
{
    struct ibv_sge sge = { .addr = 0, .length = (uint32_t)len, .lkey = mr->lkey };
    struct ibv_recv_wr wr = { .wr_id = 2, .sg_list = &sge, .num_sge = 1 };
    struct ibv_recv_wr *bad = NULL;
    int ret = ibv_post_recv(c->qp, &wr, &bad);
    if (ret) return -ret;

    struct ibv_wc wc;
    for (;;) {
        int n = ibv_poll_cq(c->cq, 1, &wc);
        if (n < 0) return -EIO;
        if (n == 0) continue;
        if (wc.status != IBV_WC_SUCCESS) return -EIO;
        return 0;
    }
}

/* Server: recv into a dmabuf, echo it straight back — zero-copy both ways. */
static int run_server(struct vconn *c)
{
    printf("[server] verbs dmabuf echo — waiting for client drive\n");
    for (int s = 0; s < g_num_sizes; s++) {
        size_t size = g_sizes[s];
        int fd = alloc_dmabuf(size);
        if (fd < 0) return 1;
        struct ibv_mr *mr = ibv_reg_dmabuf_mr(c->pd, 0, size, 0, fd,
                                IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE);
        if (!mr) {
            fprintf(stderr, "[server] reg_dmabuf_mr(%zu) failed\n", size);
            close(fd); return 1;
        }
        int total = g_warmup + g_iters;
        for (int i = 0; i < total; i++) {
            int ret = do_recv(c, mr, size);
            if (ret) { fprintf(stderr, "[server] recv size=%zu i=%d: %s\n",
                               size, i, strerror(-ret)); goto fail; }
            ret = do_send(c, mr, size);
            if (ret) { fprintf(stderr, "[server] send size=%zu i=%d: %s\n",
                               size, i, strerror(-ret)); goto fail; }
        }
        ibv_dereg_mr(mr);
        close(fd);
        printf("[server] size=%zu done (%d transfers)\n", size, total);
        continue;
fail:
        ibv_dereg_mr(mr); close(fd); return 1;
    }
    printf("[server] all sizes done\n");
    return 0;
}

/* Client: drive the ping-pong, measure RTT + one-way throughput, verify bytes. */
static int run_client(struct vconn *c)
{
    uint64_t *rtt = malloc(sizeof(uint64_t) * g_iters);
    if (!rtt) return 1;

    printf("\n  %-8s  %-11s  %-24s  %-10s  %-10s  %s\n",
           "size", "avg-rtt", "one-way-thru", "median", "p99", "integrity");
    printf("  --------------------------------------------------------"
           "--------------------------------\n");

    int any_fail = 0;
    for (int s = 0; s < g_num_sizes; s++) {
        size_t size = g_sizes[s];
        unsigned char pat = (unsigned char)(0x41 + s);

        int sfd = alloc_dmabuf(size);
        int rfd = alloc_dmabuf(size);
        if (sfd < 0 || rfd < 0) { free(rtt); return 1; }
        fill_fd(sfd, size, pat);
        fill_fd(rfd, size, 0x00);

        struct ibv_mr *smr = ibv_reg_dmabuf_mr(c->pd, 0, size, 0, sfd,
                                IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE);
        struct ibv_mr *rmr = ibv_reg_dmabuf_mr(c->pd, 0, size, 0, rfd,
                                IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE);
        if (!smr || !rmr) {
            fprintf(stderr, "[client] reg_dmabuf_mr(%zu) failed\n", size);
            free(rtt); return 1;
        }

        int fail = 0;
        for (int i = 0; i < g_warmup + g_iters; i++) {
            uint64_t t0 = now_ns();
            /* Send first, then receive the echo — same ordering as the raw
             * dmabuf bench.  do_send/do_recv each block on their own worker,
             * so recv-first would deadlock (we'd wait for an echo that can't
             * come until we send).  The server completes its recv and starts
             * its echo only after our send lands, by which time our recv is
             * posted, so the echo's RX frames have somewhere to land. */
            int ret = do_send(c, smr, size);
            if (!ret) ret = do_recv(c, rmr, size);
            if (ret) {
                fprintf(stderr, "[client] xfer size=%zu i=%d: %s\n",
                        size, i, strerror(-ret));
                fail = 1; break;
            }
            if (i >= g_warmup)
                rtt[i - g_warmup] = now_ns() - t0;
        }

        size_t bad = fail ? size : verify_fd(rfd, size, pat);
        ibv_dereg_mr(smr); ibv_dereg_mr(rmr);
        close(sfd); close(rfd);
        if (fail) { free(rtt); return 1; }

        qsort(rtt, g_iters, sizeof(uint64_t), cmp_u64);
        uint64_t med = rtt[g_iters / 2];
        uint64_t p99 = rtt[(g_iters * 99) / 100];
        uint64_t sum = 0;
        for (int i = 0; i < g_iters; i++) sum += rtt[i];
        double avg_ns = (double)sum / g_iters;
        double oneway_gbps = (double)size * 8.0 / (avg_ns / 2.0);

        char szbuf[16], thru[48], integ[32];
        if (size < 1024 * 1024)
            snprintf(szbuf, sizeof(szbuf), "%zuK", size / 1024);
        else
            snprintf(szbuf, sizeof(szbuf), "%zuM", size / (1024 * 1024));
        fmt_thru(oneway_gbps, thru, sizeof(thru));
        if (bad == 0)
            snprintf(integ, sizeof(integ), "OK");
        else {
            snprintf(integ, sizeof(integ), "FAIL(%zu/%zu bad)", bad, size);
            any_fail = 1;
        }
        printf("  %-8s  %7.2f us  %-24s  %6.2f us  %6.2f us  %s\n",
               szbuf, avg_ns / 1000.0, thru, med / 1000.0, p99 / 1000.0, integ);
    }

    free(rtt);
    printf("\n  (one-way-thru = size / (avg-rtt/2); zero-copy dmabuf verbs — "
           "paths[0], synchronous, no striping)\n");
    if (any_fail) {
        printf("  *** INTEGRITY FAILURES — throughput above is meaningless "
               "until data actually transfers ***\n");
        return 1;
    }
    printf("  integrity OK — bytes verified across the link (zero-copy recv)\n");
    return 0;
}

static void usage(const char *p)
{
    fprintf(stderr, "usage: %s <server|client> [--dev N] [--iters N] "
            "[--warmup N] [--sizes a,b,c]\n", p);
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IOLBF, 0); /* line-buffer: survive SIGTERM over ssh */
    if (argc < 2) { usage(argv[0]); return 2; }
    int is_server;
    if (!strcmp(argv[1], "server")) is_server = 1;
    else if (!strcmp(argv[1], "client")) is_server = 0;
    else { usage(argv[0]); return 2; }

    for (int i = 2; i < argc; i++) {
        if (!strcmp(argv[i], "--dev") && i + 1 < argc) g_dev = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--iters") && i + 1 < argc) g_iters = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--warmup") && i + 1 < argc) g_warmup = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--sizes") && i + 1 < argc) {
            g_num_sizes = 0;
            char *tok = strtok(argv[++i], ",");
            while (tok && g_num_sizes < 16) {
                g_sizes[g_num_sizes++] = strtoull(tok, NULL, 0);
                tok = strtok(NULL, ",");
            }
        } else { usage(argv[0]); return 2; }
    }

    struct vconn c;
    memset(&c, 0, sizeof(c));
    if (conn_open(&c)) { conn_close(&c); return 1; }
    printf("[%s] device %d open, qp_num=%u — iters=%d warmup=%d\n",
           is_server ? "server" : "client", g_dev, c.qp->qp_num,
           g_iters, g_warmup);

    int ret = is_server ? run_server(&c) : run_client(&c);
    conn_close(&c);
    return ret;
}
