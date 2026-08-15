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
#include <dlfcn.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/dma-heap.h>

#include <odl_tb5/odl_tb5.h>

/* Allocator selection.  The DMA-heap path is the deterministic CPU one used
 * by the readiness gate's main proof.  The amdgpu/HIP paths export a REAL
 * VRAM DMA-BUF through the ROCm driver stack (libdrm_amdgpu) and optionally
 * import it into HIP for fill/verify; they are discovered at runtime via
 * dlopen so this binary links nothing GPU-specific and builds everywhere.
 *
 * When an allocator's prerequisites are missing the bench prints a SKIP
 * reason and exits 3.  It NEVER silently falls back to DMA-heap or memfd —
 * a "passed" run must be backed by the allocator the user asked for. */
enum { ALLOC_DMAHEAP = 0, ALLOC_AMDGPU = 1, ALLOC_HIP = 2 };
enum { SKIP_RC = 3 };

static int g_alloc = ALLOC_DMAHEAP;
static int g_skip = 0;
static char g_skip_reason[256];

/* ── libdrm_amdgpu via dlopen (no link dependency) ───────────────────── */
struct amdgpu_bo_alloc_request {
	uint64_t alloc_size;
	uint64_t phys_alignment;
	uint64_t preferred_heap;
	uint64_t flags;
};
#define AMDGPU_GEM_DOMAIN_VRAM         0x1
#define AMDGPU_GEM_CREATE_CPU_ACCESS_REQUIRED 0x2000
#define AMDGPU_BO_HANDLE_TYPE_DMA_BUF_FD 2

struct amdgpu_api {
	void *dl;
	int (*device_initialize)(int, uint32_t *, uint32_t *, void **);
	void (*device_deinitialize)(void *);
	int (*bo_alloc)(void *, struct amdgpu_bo_alloc_request *, void **);
	void (*bo_free)(void *);
	int (*bo_export)(void *, int, uint32_t *);
};
static struct amdgpu_api g_amd;

/* ── HIP runtime via dlopen (mirrors the CUDA external-memory ABI HIP
 *    guarantees; only reached when libamdhip64 is present) ───────────── */
struct hip_ext_mem_handle {
	int type;
	union { int fd; void *win32; const void *name; } handle;
	size_t size;
	unsigned int flags;
};
struct hip_ext_mem_buffer {
	size_t offset;
	size_t size;
	unsigned int flags;
};
#define HIP_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD 1
#define HIP_MEMCPY_HOST_TO_DEVICE 0
#define HIP_MEMCPY_DEVICE_TO_HOST 1

struct hip_api {
	void *dl;
	int (*get_device_count)(int *);
	int (*import_ext_mem)(void **, const struct hip_ext_mem_handle *);
	int (*ext_mem_get_mapped)(void **, void *,
				  const struct hip_ext_mem_buffer *);
	int (*destroy_ext_mem)(void *);
	int (*memcpy)(void *, const void *, size_t, int);
	int (*sync)(void);
};
static struct hip_api g_hip;

/* Live exported fds: for amdgpu/HIP allocators close must also release the
 * amdgpu BO / HIP external-memory binding, not just the fd. */
struct live_fd {
	int fd;
	int drmfd;
	void *dev;    /* amdgpu_device_handle */
	void *bo;     /* amdgpu_bo_handle */
	void *ext;    /* hipExternalMemory_t */
	void *map;    /* mapped HIP device pointer */
};
static struct live_fd g_live[16];
static int g_live_n = 0;

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
static int alloc_dmabuf_heap(size_t size)
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

static void live_register(int fd, int drmfd, void *dev, void *bo,
			  void *ext, void *map)
{
	if (g_live_n < (int)(sizeof(g_live) / sizeof(g_live[0])))
		g_live[g_live_n++] =
			(struct live_fd){ fd, drmfd, dev, bo, ext, map };
}

static struct live_fd *live_lookup(int fd)
{
	for (int i = 0; i < g_live_n; i++)
		if (g_live[i].fd == fd)
			return &g_live[i];
	return NULL;
}

static void live_drop(int fd)
{
	for (int i = 0; i < g_live_n; i++) {
		if (g_live[i].fd != fd)
			continue;
		for (int j = i; j < g_live_n - 1; j++)
			g_live[j] = g_live[j + 1];
		g_live_n--;
		return;
	}
}

static void put_fd(int fd); /* defined below; used by the hip allocator */

static void *dlopen_any(const char **names, const char **errs,
			const char *what)
{
	for (int i = 0; names[i]; i++) {
		void *dl = dlopen(names[i], RTLD_NOW | RTLD_LOCAL);
		if (dl)
			return dl;
	}
	snprintf(g_skip_reason, sizeof(g_skip_reason),
		 "%s library not found (%s)", what, errs[0]);
	g_skip = 1;
	return NULL;
}

static void *dlsym_or_skip(void *dl, const char *sym, const char *what)
{
	void *fn = dlsym(dl, sym);
	if (!fn) {
		snprintf(g_skip_reason, sizeof(g_skip_reason),
			 "%s symbol %s missing: %s", what, sym,
			 dlerror() ? dlerror() : "?");
		g_skip = 1;
	}
	return fn;
}

/* Load libdrm_amdgpu and validate there is a live amdgpu DRM device.
 * /dev/amdgpu is a udev alias of the card node; the DRM card is what
 * libdrm actually consumes, so we probe /dev/dri/card{0..3}. */
static int probe_amdgpu(void)
{
	static const char *libs[] = { "libdrm_amdgpu.so.1",
				      "libdrm_amdgpu.so", NULL };
	const char *errs[] = { "libdrm_amdgpu.so.1", "libdrm_amdgpu.so" };
	g_amd.dl = dlopen_any(libs, errs, "amdgpu");
	if (!g_amd.dl)
		return -1;

	g_amd.device_initialize = dlsym_or_skip(g_amd.dl, "amdgpu_device_initialize", "amdgpu");
	g_amd.device_deinitialize = dlsym_or_skip(g_amd.dl, "amdgpu_device_deinitialize", "amdgpu");
	g_amd.bo_alloc = dlsym_or_skip(g_amd.dl, "amdgpu_bo_alloc", "amdgpu");
	g_amd.bo_free = dlsym_or_skip(g_amd.dl, "amdgpu_bo_free", "amdgpu");
	g_amd.bo_export = dlsym_or_skip(g_amd.dl, "amdgpu_bo_export", "amdgpu");
	if (g_skip)
		return -1;

	for (int card = 0; card < 4; card++) {
		char path[32];
		uint32_t maj, min;
		void *dev = NULL;

		snprintf(path, sizeof(path), "/dev/dri/card%d", card);
		int drmfd = open(path, O_RDWR | O_CLOEXEC);
		if (drmfd < 0)
			continue;
		if (g_amd.device_initialize(drmfd, &maj, &min, &dev) == 0) {
			g_amd.device_deinitialize(dev);
			close(drmfd);
			return 0;
		}
		close(drmfd);
	}
	snprintf(g_skip_reason, sizeof(g_skip_reason),
		 "no amdgpu DRM device (/dev/dri/card0..3)");
	g_skip = 1;
	return -1;
}

/* Load HIP and confirm at least one device is visible. */
static int probe_hip(void)
{
	static const char *libs[] = { "libamdhip64.so.6", "libamdhip64.so.5",
				      "libamdhip64.so.4", "libamdhip64.so",
				      NULL };
	const char *errs[] = { "libamdhip64.so.6", "libamdhip64.so.5",
			       "libamdhip64.so.4", "libamdhip64.so" };
	g_hip.dl = dlopen_any(libs, errs, "HIP runtime");
	if (!g_hip.dl)
		return -1;

	g_hip.get_device_count = dlsym_or_skip(g_hip.dl, "hipGetDeviceCount", "HIP");
	g_hip.import_ext_mem = dlsym_or_skip(g_hip.dl, "hipImportExternalMemory", "HIP");
	g_hip.ext_mem_get_mapped = dlsym_or_skip(g_hip.dl, "hipExternalMemoryGetMappedBuffer", "HIP");
	g_hip.destroy_ext_mem = dlsym_or_skip(g_hip.dl, "hipDestroyExternalMemory", "HIP");
	g_hip.memcpy = dlsym_or_skip(g_hip.dl, "hipMemcpy", "HIP");
	g_hip.sync = dlsym_or_skip(g_hip.dl, "hipDeviceSynchronize", "HIP");
	if (g_skip)
		return -1;

	int count = 0;
	if (g_hip.get_device_count(&count) != 0 || count < 1) {
		snprintf(g_skip_reason, sizeof(g_skip_reason),
			 "HIP installed but no HIP-visible device (hipGetDeviceCount=%d)",
			 count);
		g_skip = 1;
		return -1;
	}
	return 0;
}

/* Export a VRAM (or, on non-APUs, a CPU-accessible) amdgpu BO as a real
 * DMA-BUF fd through the ROCm driver stack.  Requires amdgpu to be loaded;
 * /dev/dri/cardN (the libdrm device) is enough — /dev/amdgpu is optional. */
static int alloc_dmabuf_amdgpu(size_t size)
{
	int drmfd = -1;
	void *dev = NULL, *bo = NULL;
	uint32_t maj, min;

	for (int card = 0; card < 4; card++) {
		char path[32];

		snprintf(path, sizeof(path), "/dev/dri/card%d", card);
		drmfd = open(path, O_RDWR | O_CLOEXEC);
		if (drmfd < 0)
			continue;
		if (g_amd.device_initialize(drmfd, &maj, &min, &dev) == 0)
			break;
		close(drmfd);
		drmfd = -1;
	}
	if (drmfd < 0 || !dev)
		return -1;

	struct amdgpu_bo_alloc_request req = {
		.alloc_size = size,
		.phys_alignment = 0,
		.preferred_heap = AMDGPU_GEM_DOMAIN_VRAM,
		.flags = AMDGPU_GEM_CREATE_CPU_ACCESS_REQUIRED,
	};
	if (g_amd.bo_alloc(dev, &req, &bo) != 0) {
		fprintf(stderr, "FAIL: amdgpu_bo_alloc(%zu) VRAM failed\n",
			size);
		g_amd.device_deinitialize(dev);
		close(drmfd);
		return -1;
	}

	uint32_t out = 0;
	if (g_amd.bo_export(bo, AMDGPU_BO_HANDLE_TYPE_DMA_BUF_FD, &out) != 0) {
		fprintf(stderr, "FAIL: amdgpu_bo_export dma-buf failed\n");
		g_amd.bo_free(bo);
		g_amd.device_deinitialize(dev);
		close(drmfd);
		return -1;
	}

	live_register((int)out, drmfd, dev, bo, NULL, NULL);
	return (int)out;
}

/* amdgpu export + HIP import: the fd is real VRAM, the mapped pointer makes
 * it directly usable by HIP kernels/memcpy. */
static int alloc_dmabuf_hip(size_t size)
{
	int fd = alloc_dmabuf_amdgpu(size);
	if (fd < 0)
		return -1;
	struct live_fd *lf = live_lookup(fd);

	struct hip_ext_mem_handle h = {
		.type = HIP_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD,
		.handle.fd = fd,
		.size = size,
	};
	void *ext = NULL;
	if (g_hip.import_ext_mem(&ext, &h) != 0 || !ext) {
		snprintf(g_skip_reason, sizeof(g_skip_reason),
			 "hipImportExternalMemory of the exported VRAM dma-buf "
			 "failed — HIP cannot bind this allocator");
		g_skip = 1;
		put_fd(fd);
		return -1;
	}
	struct hip_ext_mem_buffer b = { .offset = 0, .size = size };
	lf->ext = ext; /* put_fd destroys it on any failure path below */
	void *map = NULL;
	if (g_hip.ext_mem_get_mapped(&map, ext, &b) != 0 || !map) {
		snprintf(g_skip_reason, sizeof(g_skip_reason),
			 "hipExternalMemoryGetMappedBuffer failed");
		g_skip = 1;
		put_fd(fd);
		return -1;
	}
	lf->map = map;
	return fd;
}

/* Dispatch allocator.  Prerequisite failures set g_skip; per-allocation
 * runtime failures are hard errors (return -1 without g_skip). */
static int alloc_dmabuf(size_t size)
{
	if (g_alloc == ALLOC_AMDGPU)
		return alloc_dmabuf_amdgpu(size);
	if (g_alloc == ALLOC_HIP)
		return alloc_dmabuf_hip(size);
	return alloc_dmabuf_heap(size);
}

/* Release an allocated fd: for amdgpu/HIP also drop the BO + HIP binding. */
static void put_fd(int fd)
{
	struct live_fd *lf = live_lookup(fd);
	if (lf) {
		if (lf->ext)
			g_hip.destroy_ext_mem(lf->ext);
		g_amd.bo_free(lf->bo);
		g_amd.device_deinitialize(lf->dev);
		close(lf->drmfd);
		live_drop(fd);
	}
	close(fd);
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

/* The amdgpu exporter refuses to attach/map its DMA-BUF for a non-amdgpu
 * importer (the OdinLink NHI PCI device) on some kernel/IOMMU stacks.  That
 * surfaces as a clean -EINVAL on the very first transfer.  It is a stack
 * limitation, not an OdinLink bug — report it as a SKIP so the gate stays
 * honest without inventing a pass.  Anything after the first transfer, or
 * any other errno, stays a hard failure. */
static int is_exporter_refusal(int size_idx, int iter, int ret)
{
	return (g_alloc == ALLOC_AMDGPU || g_alloc == ALLOC_HIP) &&
	       size_idx == 0 && iter == 0 && ret == -EINVAL;
}

static void mark_exporter_skip(void)
{
	if (g_skip)
		return;
	snprintf(g_skip_reason, sizeof(g_skip_reason),
		 "amdgpu exporter refuses to map its DMA-BUF for the "
		 "OdinLink NHI DMA device on this kernel/IOMMU stack");
	g_skip = 1;
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
				if (is_exporter_refusal(s, i, ret))
					mark_exporter_skip();
				put_fd(fd);
				return 1;
			}
			ret = odl_tb5_send_dmabuf(h, fd, 0, size);
			if (ret < 0) {
				fprintf(stderr,
					"[server] send_dmabuf size=%zu i=%d: %s\n",
					size, i, strerror(-ret));
				if (is_exporter_refusal(s, i, ret))
					mark_exporter_skip();
				put_fd(fd);
				return 1;
			}
		}
		put_fd(fd);
		printf("[server] size=%zu done (%d transfers)\n", size, total);
	}
	printf("[server] all sizes done\n");
	return 0;
}

/* Fill a dmabuf.  DMA-heap buffers are CPU-mappable; amdgpu VRAM is filled
 * through the exported fd's mmap (amdgpu GEM maps VRAM for CPU access), and
 * the HIP path copies through the imported device pointer. */
static int fill_fd(int fd, size_t size, unsigned char byte)
{
	if (g_alloc == ALLOC_HIP) {
		struct live_fd *lf = live_lookup(fd);
		unsigned char *h = malloc(size);
		if (!h)
			return -1;
		memset(h, byte, size);
		g_hip.memcpy(lf->map, h, size, HIP_MEMCPY_HOST_TO_DEVICE);
		g_hip.sync();
		free(h);
		return 0;
	}
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
	if (g_alloc == ALLOC_HIP) {
		struct live_fd *lf = live_lookup(fd);
		unsigned char *h = malloc(size);
		size_t bad = 0;
		if (!h)
			return size;
		g_hip.memcpy(h, lf->map, size, HIP_MEMCPY_DEVICE_TO_HOST);
		g_hip.sync();
		for (size_t i = 0; i < size; i++)
			if (h[i] != expect)
				bad++;
		free(h);
		return bad;
	}
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
		put_fd(fd);
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
				if (is_exporter_refusal(s, i, ret))
					mark_exporter_skip();
				fail = 1;
				break;
			}
			ret = odl_tb5_recv_dmabuf(h, rfd, 0, size);
			if (ret < 0) {
				fprintf(stderr,
					"[client] recv size=%zu i=%d: %s\n",
					size, i, strerror(-ret));
				if (is_exporter_refusal(s, i, ret))
					mark_exporter_skip();
				fail = 1;
				break;
			}
			if (i >= g_warmup)
				rtt[i - g_warmup] = now_ns() - t0;
		}

		size_t bad = fail ? size : verify_fd(rfd, size, pat);
		put_fd(sfd);
		put_fd(rfd);
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
		"[--warmup N] [--sizes a,b,c] [--expect-reject a,b,c] "
		"[--allocator dmaheap|amdgpu|hip]\n"
		"  --sizes accepts K/M/G suffixes (1024-based), e.g. "
		"64K,1M,8M — or plain byte counts.\n"
		"  --expect-reject: sizes to probe ONCE (client) before the "
		"sizes loop, to arm a ring-full capacity reject before the "
		"real transfers (F2 trigger shape).\n"
		"  --allocator: backing store for the dmabufs.\n"
		"    dmaheap  system DMA-heap, CPU memory (default; the "
		"deterministic gate path)\n"
		"    amdgpu   real VRAM DMA-BUF exported via libdrm_amdgpu "
		"(needs a loaded amdgpu DRM device)\n"
		"    hip      amdgpu VRAM DMA-BUF imported into HIP for "
		"fill/verify (needs a ROCm HIP runtime too)\n"
		"  amdgpu/hip NEVER fall back: when their prerequisites are "
		"missing the bench prints a reason and exits 3 (SKIP).\n", p);
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
		} else if (!strcmp(argv[i], "--allocator") && i + 1 < argc) {
			const char *a = argv[++i];
			if (!strcmp(a, "dmaheap"))
				g_alloc = ALLOC_DMAHEAP;
			else if (!strcmp(a, "amdgpu"))
				g_alloc = ALLOC_AMDGPU;
			else if (!strcmp(a, "hip"))
				g_alloc = ALLOC_HIP;
			else {
				fprintf(stderr,
					"--allocator: unknown '%s' (expected "
					"dmaheap|amdgpu|hip)\n", a);
				return 2;
			}
		} else {
			usage(argv[0]);
			return 2;
		}
	}

	/* Early capability probe: amdgpu/HIP allocators skip cleanly (exit 3)
	 * before touching the device when their stack is missing. */
	const char *alloc_name = g_alloc == ALLOC_DMAHEAP ? "dmaheap" :
				(g_alloc == ALLOC_AMDGPU ? "amdgpu" : "hip");
	if (g_alloc == ALLOC_AMDGPU && probe_amdgpu() < 0)
		goto skip;
	if (g_alloc == ALLOC_HIP &&
	    (probe_amdgpu() < 0 || probe_hip() < 0))
		goto skip;
	if (g_skip)
		goto skip;

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
	printf("[%s] device %d open, peer ready — iters=%d warmup=%d "
	       "allocator=%s\n",
	       is_server ? "server" : "client", g_dev, g_iters, g_warmup,
	       alloc_name);

	ret = is_server ? run_server(h) : run_client(h);

	odl_tb5_close(h);
	if (ret != 0 && g_skip) {
		printf("[%s] SKIP: %s\n", is_server ? "server" : "client",
		       g_skip_reason);
		return SKIP_RC;
	}
	return ret;

skip:
	printf("[%s] SKIP: %s\n", is_server ? "server" : "client",
	       g_skip_reason[0] ? g_skip_reason : "allocator unavailable");
	return SKIP_RC;
}
