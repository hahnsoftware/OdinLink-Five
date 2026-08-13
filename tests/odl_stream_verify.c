/*
 * OdinLink — Stream Payload Integrity Verifier (odl_stream_verify)
 *
 * The transport-level answer to "do the bytes actually arrive": the CLI
 * tests (bandwidth/latency/jitter/mimo) never compare a payload byte, so
 * a transport delivering wrong data at full speed passes all of them.
 * This tool verifies CONTENT on the stream/CLI path, the analogue of the
 * thunderbolt-ibverbs `rc_write_verify` the gate suite is ported from.
 *
 * Protocol: a multi-stream echo.  Each side opens N streams; the client
 * drives R rounds per stream, sending a buffer seeded with a pattern
 * keyed by (logical stream index, round) and receiving the echo back.
 * The server, per stream, re-poisons its buffer before EVERY receive
 * (so a "never delivered" failure cannot masquerade as a pass), verifies
 * the content it received (its RX leg), then echoes it back to whoever
 * sent it (src_id), which the client verifies in turn (its RX leg).
 * All four legs of the link — client TX, server RX, server TX, client RX
 * — are content-checked, concurrently, across multiple streams, because
 * concurrency is a correctness variable here, not a performance one.
 *
 * Failure classes caught:
 *   - fragment reorder / duplicate / misattribution under the 8-byte
 *     stream header (frag_idx reassembly); counters stay clean while
 *     bytes are wrong
 *   - stale data from a previous round (round key in the seed)
 *   - cross-stream cross-talk (stream key in the seed)
 *   - intra-message shift (byte offset key in the seed)
 *   - non-delivery / short delivery (actual_len check + re-poisoning)
 *
 * Sizing: payloads are fragmented into ~4024-byte pieces by the driver,
 * so a 1 MiB payload exercises ~256 reassembly operations per round.
 *
 * The client and server MUST agree on stream ids: both open N streams in
 * the same order and the driver's id allocator is deterministic per
 * device (observed ids 20..23 for MIMO), so stream k on one end pairs
 * with stream k on the other.
 *
 * Build: linked into tests/ (see tests/CMakeLists.txt)
 * Run:   server:  ./odl_stream_verify server [--streams N] [--rounds R]
 *                 [--size S]
 *        client:  ./odl_stream_verify client [same options] [--dev N]
 *        Sizes accept K/M/G suffixes.  Gate wrapper: odl-gate-integrity.sh
 *
 * Exit: 0 = all content verified; 1 = any mismatch / short / error.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>

#include <odl_tb5/odl_tb5.h>

/* ------------------------------------------------------------------ */
/* config                                                             */
/* ------------------------------------------------------------------ */

static int    g_streams = 4;      /* concurrent streams (ids 20..)     */
static int    g_rounds  = 8;      /* echo rounds per stream            */
static size_t g_len     = (size_t)1 << 20; /* payload per message      */
static int    g_dev     = 0;

/* ------------------------------------------------------------------ */
/* pattern generation — keyed by (stream index, round, byte offset)   */
/* ------------------------------------------------------------------ */

static uint8_t pat_byte(int k, int round, size_t i)
{
	/* Knuth-style mixing: every input bit affects the high output
	 * byte, so a one-byte shift, a wrong stream, or a stale round all
	 * fail distinctly. */
	uint32_t x = (uint32_t)i + 0x9e3779b9u;
	x ^= (uint32_t)k * 2654435761u;
	x += (uint32_t)round * 2246822519u;
	x *= 3266489917u;
	x ^= x >> 16;
	return (uint8_t)(x >> 24);
}

static void fill_pattern(void *buf, size_t len, int k, int round)
{
	uint8_t *b = buf;
	for (size_t i = 0; i < len; i++)
		b[i] = pat_byte(k, round, i);
}

/* Returns number of mismatching bytes (0 = clean). */
static size_t verify_pattern(const void *buf, size_t len, int k, int round)
{
	const uint8_t *b = buf;
	size_t bad = 0;
	for (size_t i = 0; i < len; i++)
		if (b[i] != pat_byte(k, round, i))
			bad++;
	return bad;
}

/* ------------------------------------------------------------------ */
/* per-stream work                                                     */
/* ------------------------------------------------------------------ */

struct job {
	int       k;        /* logical stream index (0..streams-1)   */
	uint8_t   sid;      /* driver stream id                      */
	int       errs;     /* send/recv error count                 */
	int       shorts;   /* short-delivery count                  */
	size_t    bad_bytes;/* total mismatching bytes               */
	int       rounds_done;
};

static odl_tb5_t g_h;
static pthread_mutex_t g_report_lock = PTHREAD_MUTEX_INITIALIZER;

static void report(const char *who, int k, int r, uint8_t sid, int rc,
		   uint32_t actual, size_t bad)
{
	pthread_mutex_lock(&g_report_lock);
	if (rc < 0)
		printf("%s[%d] r=%d sid=%u ERROR %s\n", who, k, r, sid,
		       strerror(-rc));
	else if (bad)
		printf("%s[%d] r=%d sid=%u CORRUPT: %zu/%zu bytes bad\n",
		       who, k, r, sid, bad, (size_t)actual);
	else
		printf("%s[%d] r=%d sid=%u OK (%u bytes)\n", who, k, r,
		       sid, actual);
	pthread_mutex_unlock(&g_report_lock);
}

/* Client: send pattern, echo-verify. */
static void *client_worker(void *arg)
{
	struct job *j = arg;
	uint8_t *send = malloc(g_len);
	uint8_t *recv = malloc(g_len);
	if (!send || !recv) {
		fprintf(stderr, "client[%d]: OOM\n", j->k);
		exit(1);
	}

	for (int r = 0; r < g_rounds; r++) {
		uint8_t src;
		uint32_t actual;
		int rc;

		fill_pattern(send, g_len, j->k, r);

		rc = odl_tb5_stream_send(g_h, j->sid, j->sid, send, g_len);
		if (rc < 0) {
			report("client", j->k, r, j->sid, rc, 0, 0);
			j->errs++;
			break;
		}

		/* re-poison the receive side: a "never delivered" that
		 * leaves the old pattern must fail, not pass */
		memset(recv, 0x5a, g_len);

		rc = odl_tb5_stream_recv(g_h, j->sid, recv, g_len, &src,
					 &actual);
		if (rc < 0) {
			report("client", j->k, r, j->sid, rc, 0, 0);
			j->errs++;
			break;
		}
		if (actual != g_len) {
			report("client", j->k, r, j->sid, 0, actual, 0);
			printf("client[%d] r=%d SHORT: got %u want %zu\n",
			       j->k, r, actual, g_len);
			j->shorts++;
			continue;
		}

		j->bad_bytes += verify_pattern(recv, g_len, j->k, r);
		report("client", j->k, r, j->sid, 0, actual, j->bad_bytes);
		j->rounds_done++;
	}
	free(send);
	free(recv);
	return NULL;
}

/* Server: verify received content (RX leg), echo back to src. */
static void *server_worker(void *arg)
{
	struct job *j = arg;
	uint8_t *buf = malloc(g_len);
	if (!buf) {
		fprintf(stderr, "server[%d]: OOM\n", j->k);
		exit(1);
	}

	for (int r = 0; r < g_rounds; r++) {
		uint8_t src;
		uint32_t actual;
		int rc;

		/* re-poison before EVERY receive, per round */
		memset(buf, 0xa5, g_len);

		rc = odl_tb5_stream_recv(g_h, j->sid, buf, g_len, &src,
					 &actual);
		if (rc < 0) {
			report("server", j->k, r, j->sid, rc, 0, 0);
			j->errs++;
			break;
		}
		if (actual != g_len) {
			report("server", j->k, r, j->sid, 0, actual, 0);
			printf("server[%d] r=%d SHORT: got %u want %zu\n",
			       j->k, r, actual, g_len);
			j->shorts++;
			continue;
		}

		j->bad_bytes += verify_pattern(buf, actual, j->k, r);
		report("server", j->k, r, j->sid, 0, actual, j->bad_bytes);

		/* echo back to whoever sent it */
		rc = odl_tb5_stream_send(g_h, j->sid, src, buf, actual);
		if (rc < 0) {
			report("server", j->k, r, j->sid, rc, 0, 0);
			j->errs++;
			break;
		}
		j->rounds_done++;
	}
	free(buf);
	return NULL;
}

/* ------------------------------------------------------------------ */
/* CLI                                                                */
/* ------------------------------------------------------------------ */

static void usage(const char *p)
{
	fprintf(stderr,
		"usage: %s <server|client> [--dev N] [--streams N] "
		"[--rounds N] [--size S]\n"
		"  --size accepts K/M/G suffixes (1024-based), e.g. 1M "
		"or 64K\n", p);
}

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
		if (!strcmp(argv[i], "--dev") && i + 1 < argc) {
			g_dev = atoi(argv[++i]);
		} else if (!strcmp(argv[i], "--streams") && i + 1 < argc) {
			g_streams = atoi(argv[++i]);
			if (g_streams < 1 || g_streams > 255) {
				fprintf(stderr, "--streams: 1..255\n");
				return 2;
			}
		} else if (!strcmp(argv[i], "--rounds") && i + 1 < argc) {
			g_rounds = atoi(argv[++i]);
			if (g_rounds < 1) {
				fprintf(stderr, "--rounds: >= 1\n");
				return 2;
			}
		} else if (!strcmp(argv[i], "--size") && i + 1 < argc) {
			if (parse_size(argv[++i], &g_len) < 0) {
				fprintf(stderr, "--size: bad value\n");
				return 2;
			}
		} else {
			usage(argv[0]);
			return 2;
		}
	}

	int ret = odl_tb5_open(&g_h, g_dev);
	if (ret < 0) {
		fprintf(stderr, "odl_tb5_open(%d): %s\n", g_dev,
			strerror(-ret));
		return 1;
	}
	ret = odl_tb5_wait_peer(g_h, 5000);
	if (ret < 0) {
		fprintf(stderr, "odl_tb5_wait_peer: %s (peer connected?)\n",
			strerror(-ret));
		odl_tb5_close(g_h);
		return 1;
	}
	printf("[%s] streams=%d rounds=%d size=%zu — peer ready\n",
	       is_server ? "server" : "client", g_streams, g_rounds, g_len);

	/* open streams in lockstep order so ids pair across the link */
	struct job jobs[255];
	pthread_t th[255];

	for (int k = 0; k < g_streams; k++) {
		uint8_t sid = 0;
		ret = odl_tb5_stream_open(g_h, 0, &sid);
		if (ret < 0) {
			fprintf(stderr, "stream_open[%d]: %s\n", k,
				strerror(-ret));
			goto out;
		}
		jobs[k].k = k;
		jobs[k].sid = sid;
		jobs[k].errs = 0;
		jobs[k].shorts = 0;
		jobs[k].bad_bytes = 0;
		jobs[k].rounds_done = 0;
	}

	for (int k = 0; k < g_streams; k++) {
		void *(*fn)(void *) = is_server ? server_worker : client_worker;
		if (pthread_create(&th[k], NULL, fn, &jobs[k]) != 0) {
			fprintf(stderr, "pthread_create[%d]: %s\n", k,
				strerror(errno));
			ret = 1;
			goto out;
		}
	}
	for (int k = 0; k < g_streams; k++)
		pthread_join(th[k], NULL);

	/* close streams */
	for (int k = 0; k < g_streams; k++)
		odl_tb5_stream_close(g_h, jobs[k].sid);

	printf("\n=== %s summary ===\n", is_server ? "server" : "client");
	int failures = 0;
	size_t total_bad = 0;
	for (int k = 0; k < g_streams; k++) {
		printf("  stream[%d] sid=%u rounds=%d errs=%d shorts=%d bad=%zu\n",
		       k, jobs[k].sid, jobs[k].rounds_done, jobs[k].errs,
		       jobs[k].shorts, jobs[k].bad_bytes);
		total_bad += jobs[k].bad_bytes;
		if (jobs[k].errs || jobs[k].shorts || jobs[k].bad_bytes)
			failures++;
	}
	if (failures == 0 && total_bad == 0) {
		printf("INTEGRITY OK — all %d streams x %d rounds content-verified\n",
		       g_streams, g_rounds);
		ret = 0;
	} else {
		printf("INTEGRITY FAIL — %d stream(s) failed, %zu bad bytes\n",
		       failures, total_bad);
		ret = 1;
	}
out:
	odl_tb5_close(g_h);
	return ret;
}