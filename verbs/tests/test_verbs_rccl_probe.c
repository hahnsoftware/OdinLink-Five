/* Probe the discovery calls made by modern RCCL/NCCL InfiniBand code. */
#include <infiniband/verbs.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>

static int fail(const char *call, int rc)
{
    fprintf(stderr, "FAIL: %s: rc=%d errno=%d (%s)\n",
            call, rc, errno, strerror(errno));
    return 1;
}

int main(int argc, char **argv)
{
    const char *wanted = argc > 1 ? argv[1] : "odl_tb5_0";
    struct ibv_device **list = NULL;
    struct ibv_context *ctx = NULL;
    struct ibv_pd *pd = NULL;
    struct ibv_cq *cq = NULL;
    struct ibv_qp *qp = NULL;
    int ndev = 0;
    int rc = 1;

    list = ibv_get_device_list(&ndev);
    if (!list)
        return fail("ibv_get_device_list", -1);

    struct ibv_device *device = NULL;
    for (int i = 0; i < ndev; i++) {
        const char *name = ibv_get_device_name(list[i]);
        if (name && strcmp(name, wanted) == 0) {
            device = list[i];
            break;
        }
    }
    if (!device) {
        fprintf(stderr, "FAIL: device %s not found among %d verbs device(s)\n",
                wanted, ndev);
        goto out;
    }

    ctx = ibv_open_device(device);
    if (!ctx) {
        fail("ibv_open_device", -1);
        goto out;
    }

    struct ibv_query_device_ex_input input;
    struct ibv_device_attr_ex device_attr;
    memset(&input, 0, sizeof(input));
    memset(&device_attr, 0, sizeof(device_attr));
    if (ibv_query_device_ex(ctx, &input, &device_attr)) {
        fail("ibv_query_device_ex", -1);
        goto out;
    }
    if (device_attr.orig_attr.phys_port_cnt != 1 ||
        device_attr.orig_attr.max_qp == 0 ||
        device_attr.orig_attr.max_qp_wr == 0 ||
        device_attr.orig_attr.max_mr == 0) {
        fprintf(stderr, "FAIL: unusable extended device limits\n");
        goto out;
    }

    struct ibv_port_attr port_attr;
    memset(&port_attr, 0, sizeof(port_attr));
    if (ibv_query_port(ctx, 1, &port_attr)) {
        fail("ibv_query_port", -1);
        goto out;
    }
    if (port_attr.state != IBV_PORT_ACTIVE) {
        fprintf(stderr, "FAIL: port is not active (state=%u)\n",
                (unsigned)port_attr.state);
        goto out;
    }

    if (port_attr.link_layer != IBV_LINK_LAYER_INFINIBAND ||
        port_attr.active_width == 0 || port_attr.active_speed == 0) {
        fprintf(stderr,
                "FAIL: RCCL would reject port (layer=%u width=%u speed=%u)\n",
                (unsigned)port_attr.link_layer,
                (unsigned)port_attr.active_width,
                (unsigned)port_attr.active_speed);
        goto out;
    }
    struct ibv_gid_entry gid_entry;
    memset(&gid_entry, 0, sizeof(gid_entry));
    if (ibv_query_gid_ex(ctx, 1, 0, &gid_entry, 0)) {
        fail("ibv_query_gid_ex", -1);
        goto out;
    }

    pd = ibv_alloc_pd(ctx);
    cq = ibv_create_cq(ctx, 32, NULL, NULL, 0);
    if (!pd || !cq) {
        fail("queue prerequisites", -1);
        goto out;
    }

    struct ibv_qp_init_attr qia;
    memset(&qia, 0, sizeof(qia));
    qia.send_cq = cq;
    qia.recv_cq = cq;
    qia.qp_type = IBV_QPT_RC;
    qia.cap.max_send_wr = 4;
    qia.cap.max_recv_wr = 28;
    qia.cap.max_send_sge = 1;
    qia.cap.max_recv_sge = 1;
    qia.cap.max_inline_data = 256;
    qp = ibv_create_qp(pd, &qia);
    if (!qp) {
        fail("ibv_create_qp", -1);
        goto out;
    }
    if (qia.cap.max_inline_data != 0) {
        fprintf(stderr, "FAIL: provider advertised unsupported inline bytes=%u\n",
                qia.cap.max_inline_data);
        goto out;
    }
    printf("PASS: %s is RCCL-probe ready: port active/IB/20Gbps, max_qp=%d, "
           "max_qp_wr=%d, max_mr=%d, gid_type=%d, inline=%u\n",
           wanted, device_attr.orig_attr.max_qp,
           device_attr.orig_attr.max_qp_wr,
           device_attr.orig_attr.max_mr, gid_entry.gid_type,
           qia.cap.max_inline_data);
    rc = 0;

out:
    if (qp)
        ibv_destroy_qp(qp);
    if (cq)
        ibv_destroy_cq(cq);
    if (pd)
        ibv_dealloc_pd(pd);
    if (ctx)
        ibv_close_device(ctx);
    ibv_free_device_list(list);
    return rc;
}
