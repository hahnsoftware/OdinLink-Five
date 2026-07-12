/*
 * OdinLink — Verbs: RDMA WRITE_WITH_IMM smoke test (two boxes)
 *
 * ib_write_bw exercises plain RDMA WRITE but never WRITE_WITH_IMM — the op
 * RCCL/NCCL's IB transport uses to signal "your buffer is filled".  This test
 * validates that path end to end: the client RDMA-writes a known pattern into
 * the server's buffer *with* an immediate, and the server must observe an
 * IBV_WC_RECV_RDMA_WITH_IMM carrying the immediate AND find the bytes in place.
 *
 * The rkey/VA rendezvous is bootstrapped over the (already working) SEND/RECV
 * path on the same QP, so no side TCP channel is needed.
 *
 *   Server (S1):  LD_PRELOAD=...  ./test_verbs_write_imm server
 *   Client (S2):  LD_PRELOAD=...  ./test_verbs_write_imm client
 *
 * Prints "PASS" / "FAIL" and exits 0 / 1.
 */
#include <infiniband/verbs.h>
#include "odl_tb5_verbs_wrapper.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#define BUF_SIZE   65536
#define IMM_MAGIC  0xCAFEBABEu

static int g_dev = 0;

struct rkey_va { uint64_t addr; uint32_t rkey; uint32_t len; };

static struct ibv_wc wait_wc(struct ibv_cq *cq, const char *what)
{
    struct ibv_wc wc;
    for (int i = 0; i < 200000; i++) {   /* ~ up to a few seconds, no hang */
        int n = ibv_poll_cq(cq, 1, &wc);
        if (n < 0) { fprintf(stderr, "poll_cq error on %s\n", what); exit(1); }
        if (n > 0) return wc;
        usleep(50);
    }
    fprintf(stderr, "TIMEOUT waiting for %s\n", what);
    exit(1);
}

int main(int argc, char **argv)
{
    int is_server = (argc > 1 && strcmp(argv[1], "server") == 0);
    if (argc < 2) { fprintf(stderr, "usage: %s server|client\n", argv[0]); return 2; }

    struct ibv_device *dev = odl_find_tb5_device(g_dev);
    if (!dev) { fprintf(stderr, "no OdinLink device %d\n", g_dev); return 1; }
    struct ibv_context *ctx = ibv_open_device(dev);
    struct ibv_pd *pd = ibv_alloc_pd(ctx);
    struct ibv_cq *cq = ibv_create_cq(ctx, 64, NULL, NULL, 0);

    struct ibv_qp_init_attr qa;
    memset(&qa, 0, sizeof(qa));
    qa.send_cq = cq; qa.recv_cq = cq; qa.qp_type = IBV_QPT_RC;
    qa.cap.max_send_wr = 16; qa.cap.max_recv_wr = 16;
    qa.cap.max_send_sge = 1; qa.cap.max_recv_sge = 1;
    struct ibv_qp *qp = ibv_create_qp(pd, &qa);
    if (!qp) { fprintf(stderr, "create_qp failed\n"); return 1; }

    /* INIT → RTR → RTS; dest_qp_num == own qp_num (both open stream 0 first). */
    struct ibv_qp_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.qp_state = IBV_QPS_INIT; attr.pkey_index = 0; attr.port_num = 1;
    attr.qp_access_flags = IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE;
    ibv_modify_qp(qp, &attr, IBV_QP_STATE | IBV_QP_PKEY_INDEX |
                  IBV_QP_PORT | IBV_QP_ACCESS_FLAGS);
    memset(&attr, 0, sizeof(attr));
    attr.qp_state = IBV_QPS_RTR; attr.path_mtu = IBV_MTU_4096;
    attr.dest_qp_num = qp->qp_num; attr.rq_psn = 0;
    attr.max_dest_rd_atomic = 1; attr.min_rnr_timer = 12;
    ibv_modify_qp(qp, &attr, IBV_QP_STATE | IBV_QP_AV | IBV_QP_PATH_MTU |
                  IBV_QP_DEST_QPN | IBV_QP_RQ_PSN |
                  IBV_QP_MAX_DEST_RD_ATOMIC | IBV_QP_MIN_RNR_TIMER);
    memset(&attr, 0, sizeof(attr));
    attr.qp_state = IBV_QPS_RTS; attr.timeout = 14; attr.retry_cnt = 7;
    attr.rnr_retry = 7; attr.sq_psn = 0; attr.max_rd_atomic = 1;
    ibv_modify_qp(qp, &attr, IBV_QP_STATE | IBV_QP_TIMEOUT | IBV_QP_RETRY_CNT |
                  IBV_QP_RNR_RETRY | IBV_QP_SQ_PSN | IBV_QP_MAX_QP_RD_ATOMIC);

    /* Data buffer (the RDMA-WRITE target) + a small exchange buffer. */
    char *buf = aligned_alloc(4096, BUF_SIZE);
    memset(buf, 0, BUF_SIZE);
    struct ibv_mr *mr = ibv_reg_mr(pd, buf, BUF_SIZE,
                                   IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE);
    struct rkey_va xbuf_tx, xbuf_rx;
    struct ibv_mr *xmr_tx = ibv_reg_mr(pd, &xbuf_tx, sizeof(xbuf_tx), IBV_ACCESS_LOCAL_WRITE);
    struct ibv_mr *xmr_rx = ibv_reg_mr(pd, &xbuf_rx, sizeof(xbuf_rx), IBV_ACCESS_LOCAL_WRITE);

    /* Rendezvous: exchange {addr,rkey,len} over SEND/RECV as a strict
     * ping-pong (client sends first, server replies).  A *simultaneous*
     * bidirectional first-contact SEND — both peers transmitting before either
     * has received — can drop a message on this transport, so we order it the
     * way real apps do (perftest/RCCL bootstrap over TCP; the dmabuf bench
     * ping-pongs). */
    struct ibv_sge sge; struct ibv_recv_wr rwr, *rbad; struct ibv_send_wr swr, *sbad;
    memset(&rwr, 0, sizeof(rwr));
    sge = (struct ibv_sge){ .addr = (uintptr_t)&xbuf_rx, .length = sizeof(xbuf_rx), .lkey = xmr_rx->lkey };
    rwr.wr_id = 1; rwr.sg_list = &sge; rwr.num_sge = 1;
    if (ibv_post_recv(qp, &rwr, &rbad)) { fprintf(stderr, "post_recv exch failed\n"); return 1; }

    xbuf_tx.addr = (uint64_t)(uintptr_t)buf;
    xbuf_tx.rkey = mr->rkey;
    xbuf_tx.len  = BUF_SIZE;
    struct ibv_sge ssge = { .addr = (uintptr_t)&xbuf_tx, .length = sizeof(xbuf_tx), .lkey = xmr_tx->lkey };
    memset(&swr, 0, sizeof(swr));
    swr.wr_id = 2; swr.sg_list = &ssge; swr.num_sge = 1;
    swr.opcode = IBV_WR_SEND; swr.send_flags = IBV_SEND_SIGNALED;

    if (is_server) {
        wait_wc(cq, "exchange recv");                                   /* client's info first */
        if (ibv_post_send(qp, &swr, &sbad)) { fprintf(stderr, "post_send exch failed\n"); return 1; }
        wait_wc(cq, "exchange send");
    } else {
        if (ibv_post_send(qp, &swr, &sbad)) { fprintf(stderr, "post_send exch failed\n"); return 1; }
        wait_wc(cq, "exchange send");
        wait_wc(cq, "exchange recv");                                   /* server's info */
    }
    printf("[%s] rendezvous: remote addr=0x%llx rkey=0x%x len=%u\n",
           is_server ? "server" : "client",
           (unsigned long long)xbuf_rx.addr, xbuf_rx.rkey, xbuf_rx.len);

    if (is_server) {
        /* Post a receive to catch the WRITE_WITH_IMM completion. */
        memset(&rwr, 0, sizeof(rwr));
        rwr.wr_id = 3; rwr.sg_list = NULL; rwr.num_sge = 0;
        if (ibv_post_recv(qp, &rwr, &rbad)) { fprintf(stderr, "post_recv imm failed\n"); return 1; }

        struct ibv_wc wc = wait_wc(cq, "WRITE_WITH_IMM");
        int ok = 1;
        if (wc.opcode != IBV_WC_RECV_RDMA_WITH_IMM) {
            printf("FAIL: opcode=%d (want RECV_RDMA_WITH_IMM=%d)\n",
                   wc.opcode, IBV_WC_RECV_RDMA_WITH_IMM); ok = 0;
        }
        if (!(wc.wc_flags & IBV_WC_WITH_IMM)) { printf("FAIL: WITH_IMM flag missing\n"); ok = 0; }
        if (wc.imm_data != IMM_MAGIC) {
            printf("FAIL: imm=0x%x (want 0x%x)\n", wc.imm_data, IMM_MAGIC); ok = 0;
        }
        /* Verify the payload landed: buf[i] == (i*13+7) & 0xff */
        size_t bad = 0, first = (size_t)-1;
        for (size_t i = 0; i < BUF_SIZE; i++) {
            unsigned char e = (unsigned char)(i * 13 + 7);
            if ((unsigned char)buf[i] != e) { if (first == (size_t)-1) first = i; bad++; }
        }
        if (bad) { printf("FAIL: %zu payload mismatches (first @ %zu)\n", bad, first); ok = 0; }
        printf("%s: imm=0x%x byte_len=%u payload=%s\n", ok ? "PASS" : "FAIL",
               wc.imm_data, wc.byte_len, bad ? "corrupt" : "intact");
        return ok ? 0 : 1;
    } else {
        /* Fill the local source buffer with the pattern, then WRITE_WITH_IMM
         * it into the server's remote buffer. */
        for (size_t i = 0; i < BUF_SIZE; i++) buf[i] = (char)(i * 13 + 7);
        struct ibv_sge wsge = { .addr = (uintptr_t)buf, .length = BUF_SIZE, .lkey = mr->lkey };
        memset(&swr, 0, sizeof(swr));
        swr.wr_id = 4; swr.sg_list = &wsge; swr.num_sge = 1;
        swr.opcode = IBV_WR_RDMA_WRITE_WITH_IMM;
        swr.send_flags = IBV_SEND_SIGNALED;
        swr.imm_data = IMM_MAGIC;
        swr.wr.rdma.remote_addr = xbuf_rx.addr;
        swr.wr.rdma.rkey        = xbuf_rx.rkey;
        if (ibv_post_send(qp, &swr, &sbad)) { fprintf(stderr, "post_send WRITE_IMM failed\n"); return 1; }
        struct ibv_wc wc = wait_wc(cq, "WRITE_IMM completion");
        int ok = (wc.status == IBV_WC_SUCCESS && wc.opcode == IBV_WC_RDMA_WRITE);
        printf("%s: client WRITE_WITH_IMM status=%d opcode=%d\n",
               ok ? "PASS" : "FAIL", wc.status, wc.opcode);
        return ok ? 0 : 1;
    }
}
