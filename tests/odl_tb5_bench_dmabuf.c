/*
 * OdinLink — DMA-buf Transport Benchmark (the real RCCL/GPU code path)
 *
 * Measures throughput and round-trip latency of the dmabuf DMA path that
 * the RCCL/NCCL plugins actually use:
 *
 *     odl_tb5_send_dmabuf(handle, fd, 0, size)   ->  submit_tx_dmabuf()
 *     odl_tb5_recv_dmabuf(handle, fd, 0, size)   ->  submit_rx_dmabuf()
 *
 * This is a *different* path from the stream benchmark (odl_tb5_cli):
 * it is device-level, synchronous, and pinned to paths[0] (no multi-path
 * striping, no pipelining, no stream headers).  So it is the honest
 * baseline for GPU<->GPU (vLLM/RCCL) transport — the CLI's 17.9 Gb/s
 * stream numbers do NOT apply here.
 *
 * The dmabuf is backed by /dev/dma_heap/system (real dmabuf, CPU memory,
 * no GPU needed) so the number is deterministic and reproducible.  A GPU
 * (amdgpu) dmabuf would exercise the same transport with VRAM as the
 * backing store; the DMA path through the NHI is identical.
 *
 * Coordination is a self-synchronising ping-pong (echo): the client sends
 * `size` bytes and waits for the server to echo them back.  Because each
 * transfer is synchronous, the server always re-posts its next recv before
 * the client issues its next send (the server's TX completion is the very
 * event the client's recv waits on) — no out-of-band control channel and
 * no RX-not-posted race.  Start the server first, then the client.
 *
 * Build:  linked into tests/ (see CMakeLists.txt)
 * Run:    server:  ./odl_tb5_bench_dmabuf server
 *         client:  ./odl_tb5_bench_dmabuf client
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/dma-heap.h>

#include <odl_tb5/odl_tb5.h>

/* Default size sweep (bytes) and iteration counts.  Capacity is 16 to match
 * the --sizes parser's cap; sizing it to the initializer (5) let >5 sizes
 * overflow into the globals below (corrupting g_iters/g_num_sizes). */
static size_t g_sizes[16] = { 65536, 262144, 1048576, 4194304, 16777216 };
static size_t g_reject[8];
static int    g_num_sizes = 5;
static int    g_reject_n  = 0;
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
			"      (need CONFIG_DMABUF_HEAPS_SYSTEM; udmabuf/GPU "
			"paths not wired in this bench)\n",
			strerror(errno));
		return -1;
	}

	struct dma_heap_allocation_data data = {
		.len	  = size,
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

static int cmp_u64(const void *a, const void *b)
{
	uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
	return (x > y) - (x < y);
}

static void fmt_thru(double gbps, char *buf, size_t n)
{
	snprintf(buf, n, "%.2f Gb/s (%.2f GB/s)", gbps, gbps / 8.0);
}

/* Server: echo `iters`+`warmup` transfers of each size back to the client. */
static int run_server(odl_tb5_t h)
{
	printf("[server] dmabuf echo — waiting for client drive\n");
	for (int s = 0; s < g_num_sizes; s++) {
		size_t size = g_sizes[s];
		int skip = 0;
		for (int r = 0; r < g_reject_n; r++)
			if (g_reject[r] == size)
				skip = 1;
		if (skip) {
			printf("[server] size=%zu is an expect-reject probe — "
			       "skipping (client never transmits it)\n", size);
			continue;
		}

		int fd = alloc_dmabuf(size);
		if (fd < 0)
			return 1;

		int total = g_warmup + g_iters;
		for (int i = 0; i < total; i++) {
			int ret = odl_tb5_recv_dmabuf(h, fd, 0, size);
			if (ret < 0) {
				fprintf(stderr,
					"[server] recv_dmabuf size=%zu i=%d: %s\n",
					size, i, strerror(-ret));
				close(fd);
				return 1;
			}
			ret = odl_tb5_send_dmabuf(h, fd, 0, size);
			if (ret < 0) {
				fprintf(stderr,
					"[server] send_dmabuf size=%zu i=%d: %s\n",
					size, i, strerror(-ret));
				close(fd);
				return 1;
			}
		}
		close(fd);
		printf("[server] size=%zu done (%d transfers)\n", size, total);
	}
	printf("[server] all sizes done\n");
	return 0;
}

/* Fill / check a dmabuf via a CPU mapping (dma_heap buffers are mappable). */
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
		return size; /* can't verify → treat as all-bad */
	const unsigned char *b = m;
	size_t bad = 0;
	for (size_t i = 0; i < size; i++)
		if (b[i] != expect)
			bad++;
	munmap(m, size);
	return bad;
}

/* Client: drive the ping-pong and measure RTT + one-way throughput.
 * Separate send/recv buffers + a per-size pattern prove the bytes actually
 * make the round trip — a no-op transport leaves the recv buffer unchanged
 * and shows up as INTEGRITY FAIL, never as a bogus throughput number. */
static int run_client(odl_tb5_t h)
{
	uint64_t *rtt = malloc(sizeof(uint64_t) * g_iters);
	if (!rtt)
		return 1;

	printf("\n  %-8s  %-11s  %-24s  %-10s  %-10s  %s\n",
	       "size", "avg-rtt", "one-way-thru", "median", "p99",
	       "integrity");
	printf("  --------------------------------------------------------"
	       "--------------------------------\n");

	int any_fail = 0;

	/* F2 trigger probe: attempt each --expect-reject size ONCE before the
	 * real transfers.  On a ring already under pressure the driver's
	 * up-front capacity pre-check answers -ENOSPC here; on an empty ring
	 * the submit may go through.  Either outcome is acceptable — the gate
	 * only cares that the FOLLOWING transfers still complete (the pre-F2
	 * hang fired exactly on that next-op-after-reject sequence), so this
	 * probe is informational and never aborts the run. */
	for (int r = 0; r < g_reject_n; r++) {
		size_t sz = g_reject[r];
		int fd = alloc_dmabuf(sz);
		if (fd < 0)
			return 1;
		int ret = odl_tb5_send_dmabuf(h, fd, 0, sz);
		printf("[client] pre-reject probe size=%zu: %s\n", sz,
		       ret < 0 ? strerror(-ret) : "submitted (ring had room)");
		close(fd);
	}

	for (int s = 0; s < g_num_sizes; s++) {
		size_t size = g_sizes[s];
		unsigned char pat = (unsigned char)(0x41 + s); /* per-size */

		int sfd = alloc_dmabuf(size);
		int rfd = alloc_dmabuf(size);
		if (sfd < 0 || rfd < 0) {
			free(rtt);
			return 1;
		}
		fill_fd(sfd, size, pat);   /* known payload */
		fill_fd(rfd, size, 0x00);  /* clear recv side */

		int fail = 0;
		for (int i = 0; i < g_warmup + g_iters; i++) {
			uint64_t t0 = now_ns();
			int ret = odl_tb5_send_dmabuf(h, sfd, 0, size);
			if (ret < 0) {
				fprintf(stderr,
					"[client] send size=%zu i=%d: %s\n",
					size, i, strerror(-ret));
				fail = 1;
				break;
			}
			ret = odl_tb5_recv_dmabuf(h, rfd, 0, size);
			if (ret < 0) {
				fprintf(stderr,
					"[client] recv size=%zu i=%d: %s\n",
					size, i, strerror(-ret));
				fail = 1;
				break;
			}
			if (i >= g_warmup)
				rtt[i - g_warmup] = now_ns() - t0;
		}

		size_t bad = fail ? size : verify_fd(rfd, size, pat);
		close(sfd);
		close(rfd);
		if (fail) {
			free(rtt);
			return 1;
		}

		qsort(rtt, g_iters, sizeof(uint64_t), cmp_u64);
		uint64_t min = rtt[0];
		uint64_t med = rtt[g_iters / 2];
		uint64_t p99 = rtt[(g_iters * 99) / 100];
		uint64_t sum = 0;
		for (int i = 0; i < g_iters; i++)
			sum += rtt[i];
		double avg_ns = (double)sum / g_iters;
		double oneway_gbps = (double)size * 8.0 / (avg_ns / 2.0);

		char szbuf[16], thru[48], integ[32];
		if (size < 1024 * 1024)
			snprintf(szbuf, sizeof(szbuf), "%zuK", size / 1024);
		else
			snprintf(szbuf, sizeof(szbuf), "%zuM",
				 size / (1024 * 1024));
		fmt_thru(oneway_gbps, thru, sizeof(thru));
		if (bad == 0)
			snprintf(integ, sizeof(integ), "OK");
		else {
			snprintf(integ, sizeof(integ), "FAIL(%zu/%zu bad)",
				 bad, size);
			any_fail = 1;
		}

		printf("  %-8s  %7.2f us  %-24s  %6.2f us  %6.2f us  %s\n",
		       szbuf, avg_ns / 1000.0, thru, med / 1000.0,
		       p99 / 1000.0, integ);
		(void)min;
	}

	free(rtt);
	printf("\n  (one-way-thru = size / (avg-rtt/2); paths[0], synchronous "
	       "dmabuf — no striping/pipelining)\n");
	if (any_fail) {
		printf("  *** INTEGRITY FAILURES — throughput numbers above are "
		       "meaningless until data actually transfers ***\n");
		return 1;
	}
	printf("  integrity OK — bytes verified across the link\n");
	return 0;
}

static void usage(const char *p)
{
	fprintf(stderr,
		"usage: %s <server|client> [--dev N] [--iters N] "
		"[--warmup N] [--sizes a,b,c] [--expect-reject a,b,c]\n"
		"  --sizes accepts K/M/G suffixes (1024-based), e.g. "
		"64K,1M,8M — or plain byte counts.\n"
		"  --expect-reject: sizes to probe ONCE (client) before the "
		"sizes loop, to arm a ring-full capacity reject before the "
		"real transfers (F2 trigger shape).\n", p);
}

/*
 * Parse one --sizes token: a byte count with an optional K/M/G suffix
 * (1024-based, case-insensitive; a trailing 'B' is allowed).
 *
 * strtoull() alone stops at the first non-digit, so "1M" silently became 1
 * BYTE and the sweep reported 0K rows with ~20 us round trips that look like
 * plausible results. Reject trailing garbage instead of guessing.
 */
static int parse_size(const char *tok, size_t *out)
{
	char *end = NULL;
	unsigned long long v;

	errno = 0;
	v = strtoull(tok, &end, 0);
	if (errno == ERANGE || end == tok)
		return -1;

	if (*end) {
		unsigned long long mult;

		switch (*end) {
		case 'k': case 'K': mult = 1024ULL; break;
		case 'm': case 'M': mult = 1024ULL * 1024; break;
		case 'g': case 'G': mult = 1024ULL * 1024 * 1024; break;
		default:            return -1;
		}
		end++;
		/* Tolerate the "KB"/"MB" spelling, but nothing beyond it. */
		if (*end == 'b' || *end == 'B')
			end++;
		if (*end)
			return -1;
		if (v > ULLONG_MAX / mult)
			return -1;
		v *= mult;
	}

	if (v == 0 || v > SIZE_MAX)
		return -1;

	*out = (size_t)v;
	return 0;
}

int main(int argc, char **argv)
{
	if (argc < 2) {
		usage(argv[0]);
		return 2;
	}
	int is_server;
	if (!strcmp(argv[1], "server"))
		is_server = 1;
	else if (!strcmp(argv[1], "client"))
		is_server = 0;
	else {
		usage(argv[0]);
		return 2;
	}

	for (int i = 2; i < argc; i++) {
		if (!strcmp(argv[i], "--dev") && i + 1 < argc)
			g_dev = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--iters") && i + 1 < argc)
			g_iters = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--warmup") && i + 1 < argc)
			g_warmup = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--sizes") && i + 1 < argc) {
			g_num_sizes = 0;
			char *tok = strtok(argv[++i], ",");
			while (tok) {
				size_t sz;

				if (g_num_sizes == 16) {
					fprintf(stderr,
						"--sizes: at most 16 sizes\n");
					return 2;
				}
				if (parse_size(tok, &sz) < 0) {
					fprintf(stderr,
						"--sizes: bad size '%s' "
						"(expected a byte count, "
						"optionally with K/M/G)\n",
						tok);
					return 2;
				}
				g_sizes[g_num_sizes++] = sz;
				tok = strtok(NULL, ",");
			}
			if (g_num_sizes == 0) {
				fprintf(stderr, "--sizes: no sizes given\n");
				return 2;
			}
		} else if (!strcmp(argv[i], "--expect-reject") && i + 1 < argc) {
			g_reject_n = 0;
			char *tok = strtok(argv[++i], ",");
			while (tok) {
				size_t sz;

				if (g_reject_n == 8 ||
				    parse_size(tok, &sz) < 0) {
					fprintf(stderr,
						"--expect-reject: bad size "
						"'%s'\n", tok);
					return 2;
				}
				g_reject[g_reject_n++] = sz;
				tok = strtok(NULL, ",");
			}
		} else {
			usage(argv[0]);
			return 2;
		}
	}

	odl_tb5_t h = NULL;
	int ret = odl_tb5_open(&h, g_dev);
	if (ret < 0) {
		fprintf(stderr, "odl_tb5_open(%d): %s\n", g_dev,
			strerror(-ret));
		return 1;
	}
	ret = odl_tb5_wait_peer(h, 5000);
	if (ret < 0) {
		fprintf(stderr, "odl_tb5_wait_peer: %s (peer connected?)\n",
			strerror(-ret));
		odl_tb5_close(h);
		return 1;
	}
	printf("[%s] device %d open, peer ready — iters=%d warmup=%d\n",
	       is_server ? "server" : "client", g_dev, g_iters, g_warmup);

	ret = is_server ? run_server(h) : run_client(h);

	odl_tb5_close(h);
	return ret;
}
