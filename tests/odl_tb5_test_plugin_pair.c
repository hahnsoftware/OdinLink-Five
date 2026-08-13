/*
 * OdinLink — Two-Box RCCL Plugin DMA-BUF Transport Test
 *
 * Drives the net_v7 plugin exactly the way RCCL does, over a real
 * Thunderbolt link, using the plugin's DMA-BUF path end to end:
 *
 *   server: listen() x N connections, accept() each, then per-connection
 *           threads post irecv() with regMrDmaBuf() handles and verify
 *           the received pattern.
 *   client: connect() x N connections (same auto-assigned stream ids on
 *           both boxes — the kernel allocates 20.. in listen/connect
 *           order, so the peer handle needs no out-of-band exchange),
 *           then per-connection threads post isend() with dmabuf
 *           handles.
 *
 * The transfer pairing is the interesting part: the kernel's dmabuf
 * rings carry no stream header, so the plugin orders TX against RX via
 * the READY control stream (id 250).  N concurrent connections posting
 * from N proxy-style threads, with sizes crossing the 4032 B cell
 * boundary, is exactly the interleaving that breaks naive post-order
 * pairing.  Pattern verification on every request catches both wrong-
 * buffer delivery and desync.
 *
 * Run:
 *   server box:  odl_tb5_test_plugin_pair server [opts]
 *   client box:  odl_tb5_test_plugin_pair client [opts]
 *   (LD_LIBRARY_PATH must include build/lib and build/rccl)
 *
 * Exit: 0 all connections/rounds passed; 1 any failed.
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/dma-heap.h>

#include "net_v7.h"

/* Import the plugin symbol directly (linked against librccl_net_odl_tb5.so),
 * same as odl_tb5_test_plugin.c. */
extern rcclNet_v7_t rcclNetPlugin_v7;

static int g_conns = 8;
static int g_rounds = 8;
static int g_dev = 0;
static const char *g_role = NULL;
static const char *g_sizes_csv = "4031,4032,4033,65536,1048576";

static int g_failures;
static pthread_mutex_t g_fail_lock = PTHREAD_MUTEX_INITIALIZER;

static void usage(const char *argv0)
{
    fprintf(stderr,
        "usage: %s <server|client> [--conns N] [--rounds N]\n"
        "       [--sizes a,b,c] [--dev N]\n"
        "  --sizes  per-round transfer sizes (csv, default 4031,4032,4033,65536,1048576)\n",
        argv0);
}

static void parse_args(int argc, char **argv)
{
    if (argc < 2) {
        usage(argv[0]);
        exit(2);
    }
    g_role = argv[1];
    if (strcmp(g_role, "server") && strcmp(g_role, "client")) {
        usage(argv[0]);
        exit(2);
    }
    for (int i = 2; i < argc; i++) {
        if (!strcmp(argv[i], "--conns") && i + 1 < argc)
            g_conns = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--rounds") && i + 1 < argc)
            g_rounds = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--sizes") && i + 1 < argc)
            g_sizes_csv = argv[++i];
        else if (!strcmp(argv[i], "--dev") && i + 1 < argc)
            g_dev = atoi(argv[++i]);
        else {
            usage(argv[0]);
            exit(2);
        }
    }
    if (g_conns < 1 || g_conns > 64) {
        fprintf(stderr, "--conns must be 1..64\n");
        exit(2);
    }
}

/* ── sizes ─────────────────────────────────────────────────────────── */

static int g_sizes[64];
static int g_nsizes;

static void parse_sizes(void)
{
    char buf[256];
    char *save = NULL;
    snprintf(buf, sizeof(buf), "%s", g_sizes_csv);
    for (char *tok = strtok_r(buf, ",", &save);
         tok && g_nsizes < (int)(sizeof(g_sizes) / sizeof(g_sizes[0]));
         tok = strtok_r(NULL, ",", &save)) {
        long v = atol(tok);
        if (v <= 0 || v > 16 * 1024 * 1024) {
            fprintf(stderr, "bad size token '%s'\n", tok);
            exit(2);
        }
        g_sizes[g_nsizes++] = (int)v;
    }
    if (g_nsizes == 0) {
        g_sizes[0] = 65536;
        g_nsizes = 1;
    }
}

/* ── dmabuf fd (dma-heap, then memfd) ─────────────────────────────── */

static int make_dmabuf_fd(size_t size)
{
    int heap_fd = open("/dev/dma_heap/system", O_RDWR | O_CLOEXEC);
    if (heap_fd >= 0) {
        struct dma_heap_allocation_data data = {
            .len = size, .fd_flags = O_CLOEXEC | O_RDWR, .heap_flags = 0,
        };
        if (ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &data) == 0) {
            close(heap_fd);
            return data.fd;
        }
        close(heap_fd);
    }
    int fd = memfd_create("odl-plugin-pair", MFD_ALLOW_SEALING);
    if (fd < 0)
        return -1;
    ftruncate(fd, (off_t)size);
    return fd;
}

static int fill_pattern(int fd, size_t size, unsigned char byte)
{
    void *addr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED)
        return -1;
    memset(addr, byte, size);
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
            fprintf(stderr, "  mismatch at %zu: got %02x expected %02x\n",
                    i, buf[i], expected);
            munmap(addr, size);
            return -1;
        }
    }
    munmap(addr, size);
    return 0;
}

/* ── shared connection state ──────────────────────────────────────── */

struct conn {
    int k;                  /* connection index (0..conns-1) */
    void *comm;             /* sendComm (client) / recvComm (server) */
    void *mh;               /* regMrDmaBuf handle */
    void *req;              /* in-flight request (client) */
    int fd;                 /* per-conn dmabuf (client: send, server: recv) */
    int ok;
};

static pthread_barrier_t g_round_barrier;
static struct conn *g_conns_arr;

/* Marker-file sync: the plugin's accept() completes without any wire
 * handshake, so the server would post irecvs (and fire READYs) before
 * the client process even started — the READYs then hit an empty pool
 * and the transfer hangs.  The client signals after its first isend is
 * posted (control stream open, reader running); the server waits. */
#define READY_MARKER "/tmp/odl_plugin_pair_ready"

static void marker_unlink(void)
{
    unlink(READY_MARKER);
}

static void marker_touch(void)
{
    int fd = open(READY_MARKER, O_WRONLY | O_CREAT, 0644);
    if (fd >= 0)
        close(fd);
}

static int marker_wait(int timeout_s)
{
    for (int i = 0; i < timeout_s * 10; i++) {
        if (access(READY_MARKER, F_OK) == 0)
            return 0;
        struct timespec ts = { .tv_sec = 0, .tv_nsec = 100000000 };
        nanosleep(&ts, NULL);
    }
    fprintf(stderr, "timeout waiting for %s\n", READY_MARKER);
    return -1;
}

static void *client_worker(void *arg)
{
    struct conn *c = arg;
    rcclNet_v7_t *p = &rcclNetPlugin_v7;

    for (int r = 0; r < g_rounds; r++) {
        int size = g_sizes[r % g_nsizes];
        unsigned char byte = (unsigned char)(c->k * g_rounds + r);
        void *data = (void *)(uintptr_t)(0x100000000ULL + c->k);
        rcclResult_t res;

        pthread_barrier_wait(&g_round_barrier);
        if (fill_pattern(c->fd, (size_t)size, byte) < 0) {
            fprintf(stderr, "conn %d: fill_pattern failed\n", c->k);
            c->ok = 0;
            break;
        }
        res = p->isend(c->comm, data, size, r, c->mh, &c->req);
        if (res != rcclSuccess) {
            fprintf(stderr, "conn %d round %d: isend rc=%d\n", c->k, r, res);
            c->ok = 0;
            break;
        }
        marker_touch();
        /* poll until done, bounded */
        for (int spin = 0; spin < 10000; spin++) {
            int done = 0, sz = 0;
            res = p->test(c->req, &done, &sz);
            if (res != rcclSuccess) {
                fprintf(stderr, "conn %d round %d: test rc=%d\n", c->k, r, res);
                c->ok = 0;
                break;
            }
            if (done)
                break;
            if (spin == 9999) {
                fprintf(stderr, "conn %d round %d: test timeout\n", c->k, r);
                c->ok = 0;
                break;
            }
            struct timespec ts = { .tv_sec = 0, .tv_nsec = 1000000 };
            nanosleep(&ts, NULL);
        }
        if (!c->ok)
            break;
    }
    return NULL;
}

static void *server_worker(void *arg)
{
    struct conn *c = arg;
    rcclNet_v7_t *p = &rcclNetPlugin_v7;

    for (int r = 0; r < g_rounds; r++) {
        int size = g_sizes[r % g_nsizes];
        unsigned char byte = (unsigned char)(c->k * g_rounds + r);
        void *data = (void *)(uintptr_t)(0x100000000ULL + c->k);
        void *req = NULL;
        int sizes[1] = { size };
        int tags[1] = { r };
        void *datas[1] = { data };
        void *mhs[1] = { c->mh };
        rcclResult_t res;

        pthread_barrier_wait(&g_round_barrier);
        res = p->irecv(c->comm, 1, datas, sizes, tags, mhs, &req);
        if (res != rcclSuccess) {
            fprintf(stderr, "conn %d round %d: irecv rc=%d\n", c->k, r, res);
            c->ok = 0;
            break;
        }
        for (int spin = 0; spin < 10000; spin++) {
            int done = 0, sz = 0;
            res = p->test(req, &done, &sz);
            if (res != rcclSuccess) {
                fprintf(stderr, "conn %d round %d: test rc=%d\n", c->k, r, res);
                c->ok = 0;
                break;
            }
            if (done) {
                if (sz != size) {
                    fprintf(stderr, "conn %d round %d: got %d want %d\n",
                            c->k, r, sz, size);
                    c->ok = 0;
                } else if (check_pattern(c->fd, (size_t)size, byte) < 0) {
                    fprintf(stderr, "conn %d round %d: pattern MISMATCH\n",
                            c->k, r);
                    c->ok = 0;
                }
                break;
            }
            if (spin == 9999) {
                fprintf(stderr, "conn %d round %d: test timeout\n", c->k, r);
                c->ok = 0;
                break;
            }
            struct timespec ts = { .tv_sec = 0, .tv_nsec = 1000000 };
            nanosleep(&ts, NULL);
        }
        if (!c->ok)
            break;
    }
    return NULL;
}

int main(int argc, char **argv)
{
    rcclNet_v7_t *p = &rcclNetPlugin_v7;
    int ndev = 0;
    rcclResult_t res;
    int failures = 0;

    parse_args(argc, argv);
    parse_sizes();
    printf("== OdinLink plugin DMA-BUF pair test (%s, conns=%d rounds=%d dev=%d) ==\n",
           g_role, g_conns, g_rounds, g_dev);

    if (p->init(NULL) != rcclSuccess) {
        fprintf(stderr, "init failed\n");
        return 1;
    }
    if (p->devices(&ndev) != rcclSuccess || ndev < 1) {
        fprintf(stderr, "no net devices\n");
        return 1;
    }

    g_conns_arr = calloc((size_t)g_conns, sizeof(struct conn));
    if (!g_conns_arr)
        return 1;
    if (pthread_barrier_init(&g_round_barrier, NULL,
                             (unsigned)g_conns) != 0)
        return 1;

    if (!strcmp(g_role, "server")) {
        /* listen for every connection; the peer handle's stream id is
         * auto-assigned and identical on the client (same kernel ida). */
        marker_unlink();
        for (int i = 0; i < g_conns; i++) {
            char handle[64] = {0};
            void *lh = NULL;
            res = p->listen(g_dev, handle, &lh);
            if (res != rcclSuccess || !lh) {
                fprintf(stderr, "listen(%d) failed rc=%d\n", i, res);
                return 1;
            }
            printf("server: listen conn %d stream_id=%u\n", i,
                   (unsigned)((uint8_t *)handle)[0]);
            for (int retry = 0; retry < 200; retry++) {
                void *rc = NULL;
                res = p->accept(lh, &rc, NULL);
                if (res == rcclSuccess && rc) {
                    g_conns_arr[i].comm = rc;
                    break;
                }
                struct timespec ts = { .tv_sec = 0, .tv_nsec = 50000000 };
                nanosleep(&ts, NULL);
            }
            if (!g_conns_arr[i].comm) {
                fprintf(stderr, "accept(%d) never succeeded\n", i);
                return 1;
            }
        }
        for (int i = 0; i < g_conns; i++) {
            g_conns_arr[i].k = i;
            g_conns_arr[i].fd = make_dmabuf_fd((size_t)g_sizes[g_nsizes - 1]);
            if (g_conns_arr[i].fd < 0) {
                fprintf(stderr, "conn %d: dmabuf fd failed\n", i);
                return 1;
            }
            res = p->regMrDmaBuf(g_conns_arr[i].comm,
                                 (void *)(uintptr_t)(0x100000000ULL + i),
                                 (size_t)g_sizes[g_nsizes - 1],
                                 NCCL_PTR_DMABUF, 0, g_conns_arr[i].fd,
                                 &g_conns_arr[i].mh);
            if (res != rcclSuccess || !g_conns_arr[i].mh) {
                fprintf(stderr, "conn %d: regMrDmaBuf rc=%d\n", i, res);
                return 1;
            }
            g_conns_arr[i].ok = 1;
        }
        pthread_t tid[64];
        if (marker_wait(60) < 0) {
            fprintf(stderr, "client never signaled readiness\n");
            return 1;
        }
        for (int i = 0; i < g_conns; i++)
            pthread_create(&tid[i], NULL, server_worker, &g_conns_arr[i]);
        for (int i = 0; i < g_conns; i++)
            pthread_join(tid[i], NULL);
        for (int i = 0; i < g_conns; i++) {
            if (!g_conns_arr[i].ok) {
                failures++;
                fprintf(stderr, "conn %d: FAIL\n", i);
            } else {
                printf("conn %d: PASS (%d rounds)\n", i, g_rounds);
            }
            p->closeRecv(g_conns_arr[i].comm);
            p->deregMr(g_conns_arr[i].comm, g_conns_arr[i].mh);
            close(g_conns_arr[i].fd);
        }
    } else {
        /* client: same auto-assigned ids in the same order as the server */
        for (int i = 0; i < g_conns; i++) {
            char handle[64] = {0};
            handle[0] = (char)(uint8_t)(20 + i);
            snprintf(handle + 1, sizeof(handle) - 1, "odl_tb5_%d", g_dev);
            for (int retry = 0; retry < 400; retry++) {
                void *sc = NULL;
                res = p->connect(g_dev, handle, &sc, NULL);
                if (res == rcclSuccess && sc) {
                    g_conns_arr[i].comm = sc;
                    break;
                }
                struct timespec ts = { .tv_sec = 0, .tv_nsec = 50000000 };
                nanosleep(&ts, NULL);
            }
            if (!g_conns_arr[i].comm) {
                fprintf(stderr, "connect(%d) never succeeded\n", i);
                return 1;
            }
            printf("client: connect conn %d -> dst %u\n", i,
                   (unsigned)(uint8_t)handle[0]);
        }
        for (int i = 0; i < g_conns; i++) {
            g_conns_arr[i].k = i;
            g_conns_arr[i].fd = make_dmabuf_fd((size_t)g_sizes[g_nsizes - 1]);
            if (g_conns_arr[i].fd < 0) {
                fprintf(stderr, "conn %d: dmabuf fd failed\n", i);
                return 1;
            }
            res = p->regMrDmaBuf(g_conns_arr[i].comm,
                                 (void *)(uintptr_t)(0x100000000ULL + i),
                                 (size_t)g_sizes[g_nsizes - 1],
                                 NCCL_PTR_DMABUF, 0, g_conns_arr[i].fd,
                                 &g_conns_arr[i].mh);
            if (res != rcclSuccess || !g_conns_arr[i].mh) {
                fprintf(stderr, "conn %d: regMrDmaBuf rc=%d\n", i, res);
                return 1;
            }
            g_conns_arr[i].ok = 1;
        }
        pthread_t tid[64];
        for (int i = 0; i < g_conns; i++)
            pthread_create(&tid[i], NULL, client_worker, &g_conns_arr[i]);
        for (int i = 0; i < g_conns; i++)
            pthread_join(tid[i], NULL);
        for (int i = 0; i < g_conns; i++) {
            if (!g_conns_arr[i].ok) {
                failures++;
                fprintf(stderr, "conn %d: FAIL\n", i);
            } else {
                printf("conn %d: PASS (%d rounds)\n", i, g_rounds);
            }
            p->closeSend(g_conns_arr[i].comm);
            p->deregMr(g_conns_arr[i].comm, g_conns_arr[i].mh);
            close(g_conns_arr[i].fd);
        }
    }

    printf("=== %s: %s (%d failures) ===\n", g_role,
           failures == 0 ? "PASS" : "FAIL", failures);
    return failures == 0 ? 0 : 1;
}