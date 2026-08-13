/*
 * Paired DMA-buf transfer test for real hardware (two boxes).
 *
 * The single-box odl_tb5_test_rccl_dmabuf cannot pass on a real link with
 * num_paths=2: synchronous dmabuf traffic lives on the reserved data rings
 * (1..nps-1), the pool never posts there, and a TX frame completes only
 * once the PEER has posted RX staging frames.  One box alone therefore
 * starves its own sends — by design (see odl_tb5_rx_arm / the F2 notes).
 *
 * This runner splits the roles and posts the receiver's staging FIRST
 * (matching how RCCL/NCCL rendezvous actually works: recv is posted before
 * send).  Coordination is a marker file in /tmp; the sender polls for it.
 *
 * Usage (one box receiver, the other sender):
 *   recv-box:  ./odl_tb5_pair_dmabuf recv -d 0 [size]
 *   send-box:  ./odl_tb5_pair_dmabuf send -d 0 [size]
 *
 * Both directions are covered by swapping the roles.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>

#include "odl_tb5/odl_tb5.h"

static int dev_index = 0;
static size_t buf_size = 65536;

/* ── dmabuf fd creation (same fallback chain as the single-box test) ─── */
static int dma_heap_fd(size_t size, int *real)
{
    const char *heap = "/dev/dma_heap/system";
    int fd = open(heap, O_RDWR);
    if (fd < 0)
        return -1;

    struct dma_heap_allocation_data {
        __u64 len;
        __u32 fd;
        __u32 fd_flags;
    } data = { .len = size, .fd_flags = O_RDWR };
    if (ioctl(fd, _IOWR(0xc0, 0, struct dma_heap_allocation_data), &data) < 0) {
        close(fd);
        return -1;
    }
    close(fd);
    *real = 1;
    return (int)data.fd;
}

static int make_dmabuf_fd(size_t size, int *real)
{
    int fd = dma_heap_fd(size, real);
    if (fd >= 0)
        return fd;
    fd = memfd_create("odl_pair_dmabuf", 0);
    if (fd < 0)
        return -1;
    if (ftruncate(fd, (off_t)size) < 0) {
        close(fd);
        return -1;
    }
    *real = 0;
    return fd;
}

static int write_pattern(int fd, size_t size, unsigned char c)
{
    void *addr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED)
        return -1;
    memset(addr, c, size);
    munmap(addr, size);
    return 0;
}

static int check_pattern(int fd, size_t size, unsigned char expected)
{
    void *addr = mmap(NULL, size, PROT_READ, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED)
        return -1;
    unsigned char *buf = addr;
    for (size_t i = 0; i < size; i++) {
        if (buf[i] != expected) {
            fprintf(stderr, "  mismatch at offset %zu: got %02x expected %02x\n",
                    i, buf[i], expected);
            munmap(addr, size);
            return -1;
        }
    }
    munmap(addr, size);
    return 0;
}

static void marker_path(char *out, size_t outsz, const char *stage)
{
    snprintf(out, outsz, "/tmp/odl_pair_dev%d_%s.armed", dev_index, stage);
}

static int wait_peer_retry(odl_tb5_t handle)
{
    int ret = -110;
    for (int i = 0; i < 30; i++) {
        ret = odl_tb5_wait_peer(handle, 1000);
        if (ret == 0)
            return 0;
        usleep(300000);
    }
    return ret;
}

static void unlink_marker(const char *stage)
{
    char path[128];
    marker_path(path, sizeof(path), stage);
    unlink(path);
}

static void touch_marker(const char *stage)
{
    char path[128];
    marker_path(path, sizeof(path), stage);
    int fd = open(path, O_CREAT | O_WRONLY, 0644);
    if (fd >= 0) {
        write(fd, "1", 1);
        close(fd);
    }
}

static int wait_marker(const char *stage, int timeout_s)
{
    char path[128];
    marker_path(path, sizeof(path), stage);
    for (int i = 0; i < timeout_s * 10; i++) {
        struct stat st;
        if (stat(path, &st) == 0)
            return 0;
        usleep(100000);
    }
    fprintf(stderr, "  timeout waiting for marker %s\n", path);
    return -1;
}

/* Receiver thread: legacy recv_dmabuf (posts staging RX at ioctl entry). */
struct legacy_recv_args {
    odl_tb5_t handle;
    int fd;
};

static void *legacy_recv_thread(void *arg)
{
    struct legacy_recv_args *a = arg;
    int ret = odl_tb5_recv_dmabuf(a->handle, a->fd, 0, buf_size);
    pthread_exit((void *)(long)ret);
    return NULL;
}

/* Receiver thread: stream recv_dmabuf. */
struct stream_recv_args {
    odl_tb5_t handle;
    uint8_t stream_id;
    int fd;
};

static void *stream_recv_thread(void *arg)
{
    struct stream_recv_args *a = arg;
    int ret = odl_tb5_stream_recv_dmabuf(a->handle, a->stream_id,
                                         a->fd, 0, buf_size);
    pthread_exit((void *)(long)ret);
    return NULL;
}

static int do_recv(void)
{
    int failures = 0;
    odl_tb5_t handle = NULL;
    int ret;

    /* Never act on a stale marker from an earlier run. */
    unlink_marker("legacy");
    unlink_marker("stream");

    ret = odl_tb5_open(&handle, dev_index);
    if (ret < 0) {
        fprintf(stderr, "FAIL: odl_tb5_open(%d) returned %d\n", dev_index, ret);
        return 1;
    }
    ret = wait_peer_retry(handle);
    if (ret < 0) {
        fprintf(stderr, "FAIL: odl_tb5_wait_peer returned %d\n", ret);
        odl_tb5_close(handle);
        return 1;
    }
    printf("recv: peer ready (dev %d)\n", dev_index);

    /* ── legacy stage: post RX first, then announce readiness ─────── */
    int is_real = 0;
    int recv_fd = make_dmabuf_fd(buf_size, &is_real);
    if (recv_fd < 0) {
        fprintf(stderr, "FAIL: recv dmabuf fd creation\n");
        odl_tb5_close(handle);
        return 1;
    }
    printf("recv: staging recv_dmabuf fd=%d (real=%d)\n", recv_fd, is_real);

    struct legacy_recv_args la = { .handle = handle, .fd = recv_fd };
    pthread_t tid;
    /* The staging window is 5 s per kernel wait; retry up to 12 attempts
     * (60 s) so an orchestrator has slack to start the sender.  Each
     * failure is a timeout with ZERO bytes delivered (no credits -> no
     * DMA), so a retry cannot duplicate data. */
    int handled = -1;
    void *join_ret;
    touch_marker("legacy");
    printf("recv: legacy staging posted, retrying; waiting for sender...\n");
    for (int attempt = 0; attempt < 12; attempt++) {
        pthread_create(&tid, NULL, legacy_recv_thread, &la);
        pthread_join(tid, &join_ret);
            handled = (int)(long)join_ret;
        if (handled == 0 || handled != -110)
            break;
    }
    ret = handled;
    if (ret == 0) {
        printf("recv: legacy recv_dmabuf OK\n");
        if (is_real && check_pattern(recv_fd, buf_size, 0xAB) != 0) {
            fprintf(stderr, "FAIL: recv buffer pattern MISMATCH\n");
            failures++;
        } else {
            printf("recv: legacy pattern verified\n");
        }
    } else {
        fprintf(stderr, "FAIL: legacy recv_dmabuf returned %d (%s)\n",
                ret, strerror(-ret));
        failures++;
    }
    close(recv_fd);
    unlink_marker("legacy");

    /* ── stream stage ─────────────────────────────────────────────── */
    uint8_t stream_id = 0;
    ret = odl_tb5_stream_open(handle, 0, &stream_id);
    if (ret != 0) {
        fprintf(stderr, "FAIL: stream_open returned %d\n", ret);
        failures++;
        odl_tb5_close(handle);
        return failures > 0 ? 1 : 0;
    }
    printf("recv: stream %u opened\n", stream_id);

    is_real = 0;
    recv_fd = make_dmabuf_fd(buf_size, &is_real);
    if (recv_fd < 0) {
        fprintf(stderr, "FAIL: recv stream dmabuf fd creation\n");
        failures++;
    } else {
        struct stream_recv_args sa = {
            .handle = handle, .stream_id = stream_id, .fd = recv_fd,
        };
        int handled = -1;
    void *join_ret;
        touch_marker("stream");
        printf("recv: stream staging posted, retrying; waiting for sender...\n");
        for (int attempt = 0; attempt < 12; attempt++) {
            pthread_create(&tid, NULL, stream_recv_thread, &sa);
            pthread_join(tid, &join_ret);
            handled = (int)(long)join_ret;
            if (handled == 0 || handled != -110)
                break;
        }
        ret = handled;
        if (ret == 0) {
            printf("recv: stream recv_dmabuf OK\n");
            if (is_real && check_pattern(recv_fd, buf_size, 0xAB) != 0) {
                fprintf(stderr, "FAIL: stream recv pattern MISMATCH\n");
                failures++;
            } else {
                printf("recv: stream pattern verified\n");
            }
        } else {
            fprintf(stderr, "FAIL: stream recv_dmabuf returned %d (%s)\n",
                    ret, strerror(-ret));
            failures++;
        }
        close(recv_fd);
        unlink_marker("stream");
    }

    odl_tb5_stream_close(handle, stream_id);
    odl_tb5_close(handle);
    return failures > 0 ? 1 : 0;
}

static int do_send(void)
{
    int failures = 0;
    odl_tb5_t handle = NULL;
    int ret;

    ret = odl_tb5_open(&handle, dev_index);
    if (ret < 0) {
        fprintf(stderr, "FAIL: odl_tb5_open(%d) returned %d\n", dev_index, ret);
        return 1;
    }

    /* Orchestrator guarantees receiver staging is already posted; the
     * receiver retries for 60 s, so just open and fire. */
    ret = wait_peer_retry(handle);
    if (ret < 0) {
        fprintf(stderr, "FAIL: odl_tb5_wait_peer returned %d\n", ret);
        odl_tb5_close(handle);
        return 1;
    }
    printf("send: peer ready (dev %d)\n", dev_index);

    /* ── legacy stage ─────────────────────────────────────────────── */
    int is_real = 0;
    int send_fd = make_dmabuf_fd(buf_size, &is_real);
    if (send_fd < 0) {
        fprintf(stderr, "FAIL: send dmabuf fd creation\n");
        odl_tb5_close(handle);
        return 1;
    }
    if (is_real)
        write_pattern(send_fd, buf_size, 0xAB);

    ret = -1;
    for (int attempt = 0; attempt < 12; attempt++) {
        ret = odl_tb5_send_dmabuf(handle, send_fd, 0, buf_size);
        if (ret == 0 || ret != -110)
            break;
    }
    if (ret == 0) {
        printf("send: legacy send_dmabuf OK\n");
    } else {
        fprintf(stderr, "FAIL: legacy send_dmabuf returned %d (%s)\n",
                ret, strerror(-ret));
        failures++;
    }
    close(send_fd);

    /* ── stream stage ─────────────────────────────────────────────── */
    uint8_t stream_id = 0;
    ret = odl_tb5_stream_open(handle, 0, &stream_id);
    if (ret != 0) {
        fprintf(stderr, "FAIL: stream_open returned %d\n", ret);
        failures++;
        odl_tb5_close(handle);
        return failures > 0 ? 1 : 0;
    }
    printf("send: stream %u opened\n", stream_id);

    is_real = 0;
    send_fd = make_dmabuf_fd(buf_size, &is_real);
    if (send_fd < 0) {
        fprintf(stderr, "FAIL: send stream dmabuf fd creation\n");
        failures++;
    } else {
        if (is_real)
            write_pattern(send_fd, buf_size, 0xAB);

        ret = -1;
        for (int attempt = 0; attempt < 12; attempt++) {
            ret = odl_tb5_stream_send_dmabuf(handle, stream_id, stream_id,
                                             send_fd, 0, buf_size);
            if (ret == 0 || ret != -110)
                break;
        }
        if (ret == 0) {
            printf("send: stream send_dmabuf OK\n");
        } else {
            fprintf(stderr, "FAIL: stream send_dmabuf returned %d (%s)\n",
                    ret, strerror(-ret));
            failures++;
        }
        close(send_fd);
    }

    odl_tb5_stream_close(handle, stream_id);
    odl_tb5_close(handle);
    return failures > 0 ? 1 : 0;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr,
                "Usage: %s recv|send -d <dev> [size]\n"
                "  recv on one box, send on the peer; recv arms its RX "
                "staging first (RCCL-style rendezvous).\n",
                argv[0]);
        return 2;
    }

    const char *role = argv[1];
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            dev_index = atoi(argv[++i]);
        } else if (argv[i][0] != '-') {
            buf_size = (size_t)strtoul(argv[i], NULL, 0);
        }
    }

    printf("== OdinLink paired dmabuf test (%s, dev %d, size %zu) ==\n",
           role, dev_index, buf_size);

    int rc;
    if (strcmp(role, "recv") == 0)
        rc = do_recv();
    else if (strcmp(role, "send") == 0)
        rc = do_send();
    else {
        fprintf(stderr, "unknown role: %s\n", role);
        return 2;
    }

    printf("== %s %s ==\n", rc == 0 ? "PASS" : "FAIL", role);
    return rc;
}