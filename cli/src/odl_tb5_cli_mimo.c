/*
 * OdinLink — CLI: MIMO Test (Multiple Streams at Once)
 *
 * Opens several streams in parallel and blasts data through all of them
 * simultaneously. Tests how well the multiplexed I/O path handles
 * concurrent traffic — relevant for NCCL collective operations where
 * multiple GPUs are sending at the same time.
 *
 * Each worker thread drives its OWN stream with a distinct stream ID.
 * This matters for two reasons:
 *   1. The driver pins a stream onto a TX path via stream_id %
 *      tx_active_paths. Distinct IDs with mixed parity therefore spread
 *      the load across both paths (real multi-path striping). A single
 *      shared ID would pin everything onto one path.
 *   2. Only one sender per stream keeps the stream-internal flow-control
 *      (tx_in_flight / tx_queue_max) from deadlocking. Multiple senders
 *      on one stream hang uninterruptibly at join.
 *
 * Client and server agree on the set of data-stream IDs implicitly:
 * both derive them as ODL_MIMO_STREAM_BASE + i for i in [0, num_streams).
 * num_streams travels to the server inside the TEST_REQ. A short ready
 * handshake (server opens its N streams, then ACKs on the control stream)
 * ensures the client does not send into filters the server has not opened.
 */
#include "odl_tb5_cli.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

/*
 * Base filter/stream ID for MIMO data streams. Chosen to avoid the
 * well-known control IDs (ODL_STREAM_TEST=1, ODL_STREAM_SYNC=2,
 * ODL_STREAM_CLI=10). Even base + i gives mixed parity across streams so
 * that, with 2 active paths, streams alternate between path 0 and path 1.
 */
#define ODL_MIMO_STREAM_BASE  20
#define ODL_MIMO_STREAM_MAX   200   /* keep base + i within uint8 range */

/* Resolve num_streams the same way on both sides. */
static uint32_t mimo_resolve_streams(uint32_t requested)
{
	uint32_t n = requested ? requested : ODL_DEFAULT_STREAMS;

	if (n > ODL_MIMO_STREAM_MAX)
		n = ODL_MIMO_STREAM_MAX;
	return n;
}

/* ── Client side ───────────────────────────────────────────────────── */

struct mimo_stream_ctx {
	odl_tb5_t     handle;
	uint8_t       sid;        /* local data stream we send FROM */
	uint8_t       dst;        /* peer data stream we send TO */
	int           stream_id;  /* logical index, for reporting */
	uint32_t      block_size;
	uint32_t      duration_sec;
	volatile bool stop;
	uint64_t      bytes_transferred;
	uint64_t      elapsed_ns;
};

/* Worker thread for a single MIMO stream (one sender per stream). */
static void *mimo_stream_thread(void *arg)
{
	struct mimo_stream_ctx *ctx = (struct mimo_stream_ctx *)arg;
	uint8_t *data;
	uint64_t t_start, t_end;
	uint64_t deadline_ns;
	int ret;

	data = malloc(ctx->block_size);
	if (!data)
		return NULL;
	memset(data, 0xAA, ctx->block_size);

	t_start = odl_time_ns();
	deadline_ns = t_start + (uint64_t)ctx->duration_sec * 1000000000ULL;

	while (!ctx->stop) {
		if (odl_time_ns() >= deadline_ns)
			break;

		ret = odl_tb5_stream_send(ctx->handle, ctx->sid, ctx->dst,
					  data, ctx->block_size);
		if (ret < 0)
			break;

		ctx->bytes_transferred += ctx->block_size;
	}

	t_end = odl_time_ns();
	ctx->elapsed_ns = t_end - t_start;

	free(data);
	return NULL;
}

/* Wait for a specific control message on the control stream. */
static int mimo_wait_ctrl(odl_tb5_t handle, uint8_t sid, uint32_t want_type)
{
	char buf[4096];
	uint32_t type, seq;
	int ret;

	for (;;) {
		ret = odl_cli_recv_msg(handle, sid, buf, sizeof(buf),
				       &type, &seq, NULL);
		if (ret == -ETIMEDOUT)
			continue;
		if (ret < 0)
			return ret;
		if (type == want_type)
			return 0;
		/* ignore unexpected control chatter */
	}
}

/* Client (initiator) for MIMO test. */
int odl_cli_mimo_client(odl_tb5_t handle, uint8_t sid, uint8_t dst,
			 const struct odl_cli_params *params)
{
	struct mimo_stream_ctx *streams = NULL;
	pthread_t *threads = NULL;
	uint32_t num_streams;
	uint32_t block_size;
	uint32_t duration_sec;
	uint32_t opened = 0;
	uint64_t total_bytes = 0;
	uint64_t max_elapsed_ns = 0;
	char msg_buf[4096];
	uint32_t type, seq;
	int ret;

	num_streams = mimo_resolve_streams(params->num_streams);
	block_size = params->block_sizes[0] ? params->block_sizes[0]
					    : ODL_DEFAULT_BLOCK_SIZE;
	duration_sec = params->duration_sec ? params->duration_sec
					    : ODL_DEFAULT_DURATION;

	char size_buf[32];
	printf("  Streams: %u\n", num_streams);
	printf("  Block size: %s\n",
	       odl_format_size(block_size, size_buf, sizeof(size_buf)));
	printf("  Duration: %u seconds\n", duration_sec);

	streams = calloc(num_streams, sizeof(*streams));
	if (!streams)
		return -ENOMEM;

	threads = calloc(num_streams, sizeof(*threads));
	if (!threads) {
		free(streams);
		return -ENOMEM;
	}

	/* Open one distinct data stream per worker. */
	for (uint32_t i = 0; i < num_streams; i++) {
		uint8_t filter = (uint8_t)(ODL_MIMO_STREAM_BASE + i);
		uint8_t data_sid;

		ret = odl_tb5_stream_open(handle, filter, &data_sid);
		if (ret < 0) {
			fprintf(stderr, "  Failed to open data stream %u: %s\n",
				i, strerror(-ret));
			goto out_close;
		}

		streams[i].handle = handle;
		streams[i].sid = data_sid;
		streams[i].dst = filter;   /* peer opens the same filter id */
		streams[i].stream_id = (int)i;
		streams[i].block_size = block_size;
		streams[i].duration_sec = duration_sec;
		streams[i].stop = false;
		streams[i].bytes_transferred = 0;
		streams[i].elapsed_ns = 0;
		opened++;
	}

	/* Tell the server to start and open its matching data streams. */
	ret = odl_cli_send_msg(handle, sid, dst,
			       ODL_CLI_MSG_TEST_START, 0, NULL, 0);
	if (ret < 0)
		goto out_close;

	/* Wait until the server has its data streams open (ready ACK). */
	ret = mimo_wait_ctrl(handle, sid, ODL_CLI_MSG_TEST_ACK);
	if (ret < 0) {
		fprintf(stderr, "  Server did not become ready: %s\n",
			strerror(-ret));
		goto out_stop;
	}

	printf("  Launching %u streams...\n", num_streams);
	for (uint32_t i = 0; i < num_streams; i++) {
		ret = pthread_create(&threads[i], NULL,
				     mimo_stream_thread, &streams[i]);
		if (ret != 0) {
			fprintf(stderr, "  Failed to create thread %u: %s\n",
				i, strerror(ret));
			for (uint32_t j = 0; j < i; j++)
				streams[j].stop = true;
			for (uint32_t j = 0; j < i; j++)
				pthread_join(threads[j], NULL);
			ret = -ret;
			goto out_stop;
		}
	}

	{
		struct timespec ts;
		ts.tv_sec = duration_sec;
		ts.tv_nsec = 0;
		nanosleep(&ts, NULL);
	}

	for (uint32_t i = 0; i < num_streams; i++)
		streams[i].stop = true;

	for (uint32_t i = 0; i < num_streams; i++)
		pthread_join(threads[i], NULL);

	printf("\n  === Per-Stream Results ===\n");
	for (uint32_t i = 0; i < num_streams; i++) {
		uint64_t bytes = streams[i].bytes_transferred;
		uint64_t ns = streams[i].elapsed_ns;

		total_bytes += bytes;
		if (ns > max_elapsed_ns)
			max_elapsed_ns = ns;

		double gbytes_s = 0.0;
		if (ns > 0)
			gbytes_s = (double)bytes / (double)ns;

		printf("  Stream %d: %.2f GB/s (%lu bytes in ",
		       streams[i].stream_id, gbytes_s,
		       (unsigned long)bytes);

		char lat_buf[64];
		printf("%s)\n", odl_format_latency(ns, lat_buf, sizeof(lat_buf)));
	}

	{
		char tp_buf[64];
		double agg_gbytes_s = 0.0;

		if (max_elapsed_ns > 0)
			agg_gbytes_s = (double)total_bytes / (double)max_elapsed_ns;

		printf("\n  === Aggregate ===\n");
		printf("  Total: %.2f GB/s across %u streams\n",
		       agg_gbytes_s, num_streams);
		printf("  Total data: %s\n",
		       odl_format_size(total_bytes, tp_buf, sizeof(tp_buf)));
		printf("  Throughput: %s\n",
		       odl_format_throughput(total_bytes, max_elapsed_ns,
					    tp_buf, sizeof(tp_buf)));
	}

	ret = 0;

out_stop:
	odl_cli_send_msg(handle, sid, dst,
			 ODL_CLI_MSG_TEST_STOP, 0, NULL, 0);

	{
		struct odl_cli_result result;

		memset(&result, 0, sizeof(result));
		result.bytes_transferred = total_bytes;
		result.elapsed_ns = max_elapsed_ns;

		odl_cli_send_msg(handle, sid, dst, ODL_CLI_MSG_RESULT, 0,
				 &result.bytes_transferred,
				 sizeof(result) - sizeof(result.hdr));

		odl_cli_recv_msg(handle, sid, msg_buf, sizeof(msg_buf),
				 &type, &seq, NULL);
	}

out_close:
	for (uint32_t i = 0; i < opened; i++)
		odl_tb5_stream_close(handle, streams[i].sid);

	free(threads);
	free(streams);

	return ret;
}

/* ── Server side ───────────────────────────────────────────────────── */

struct mimo_rx_ctx {
	odl_tb5_t     handle;
	uint8_t       sid;        /* local data stream we receive on */
	uint32_t      buf_size;
	volatile bool *stop;
	uint64_t      bytes_received;
	int           stream_id;
};

/* Receiver thread for a single MIMO data stream. */
static void *mimo_rx_thread(void *arg)
{
	struct mimo_rx_ctx *ctx = (struct mimo_rx_ctx *)arg;
	uint8_t *buf;

	buf = malloc(ctx->buf_size);
	if (!buf)
		return NULL;

	while (!*ctx->stop) {
		uint8_t src_id;
		uint32_t actual_len;
		int ret;

		ret = odl_tb5_stream_wait_rx(ctx->handle, ctx->sid, 500);
		if (ret == -ETIMEDOUT)
			continue;   /* re-check stop flag */
		if (ret < 0)
			break;

		ret = odl_tb5_stream_recv(ctx->handle, ctx->sid, buf,
					  ctx->buf_size, &src_id, &actual_len);
		if (ret < 0)
			break;

		ctx->bytes_received += actual_len;
	}

	free(buf);
	return NULL;
}

/* Server (responder) for MIMO test. */
int odl_cli_mimo_server(odl_tb5_t handle, uint8_t sid, uint8_t dst,
			 const struct odl_cli_test_req *req)
{
	struct mimo_rx_ctx *rx = NULL;
	pthread_t *threads = NULL;
	uint32_t num_streams;
	uint32_t block_size;
	uint32_t buf_size;
	uint32_t opened = 0;
	volatile bool stop = false;
	char msg_buf[4096];
	uint32_t type, seq;
	uint64_t bytes_received = 0;
	uint64_t t_start, t_end = 0;
	int ret;

	num_streams = mimo_resolve_streams(req->num_streams);
	block_size = req->block_size ? req->block_size : ODL_DEFAULT_BLOCK_SIZE;
	buf_size = block_size > 4096 ? block_size : 4096;

	/* Wait for the client's TEST_START on the control stream. */
	ret = mimo_wait_ctrl(handle, sid, ODL_CLI_MSG_TEST_START);
	if (ret < 0)
		return ret;

	printf("  [Server] MIMO test starting (%u streams)\n", num_streams);

	rx = calloc(num_streams, sizeof(*rx));
	if (!rx)
		return -ENOMEM;

	threads = calloc(num_streams, sizeof(*threads));
	if (!threads) {
		free(rx);
		return -ENOMEM;
	}

	/* Open the matching data streams before we ACK ready. */
	for (uint32_t i = 0; i < num_streams; i++) {
		uint8_t filter = (uint8_t)(ODL_MIMO_STREAM_BASE + i);
		uint8_t data_sid;

		ret = odl_tb5_stream_open(handle, filter, &data_sid);
		if (ret < 0) {
			fprintf(stderr,
				"  [Server] Failed to open data stream %u: %s\n",
				i, strerror(-ret));
			goto out_close;
		}

		rx[i].handle = handle;
		rx[i].sid = data_sid;
		rx[i].buf_size = buf_size;
		rx[i].stop = &stop;
		rx[i].bytes_received = 0;
		rx[i].stream_id = (int)i;
		opened++;
	}

	t_start = odl_time_ns();

	for (uint32_t i = 0; i < num_streams; i++) {
		ret = pthread_create(&threads[i], NULL,
				     mimo_rx_thread, &rx[i]);
		if (ret != 0) {
			fprintf(stderr,
				"  [Server] Failed to create rx thread %u: %s\n",
				i, strerror(ret));
			stop = true;
			for (uint32_t j = 0; j < i; j++)
				pthread_join(threads[j], NULL);
			ret = -ret;
			goto out_close;
		}
	}

	/* Signal readiness so the client starts sending. */
	ret = odl_cli_send_msg(handle, sid, dst,
			       ODL_CLI_MSG_TEST_ACK, 0, NULL, 0);
	if (ret < 0) {
		stop = true;
		for (uint32_t i = 0; i < num_streams; i++)
			pthread_join(threads[i], NULL);
		goto out_close;
	}

	/* Run until the client says stop (on the control stream). */
	ret = mimo_wait_ctrl(handle, sid, ODL_CLI_MSG_TEST_STOP);
	t_end = odl_time_ns();

	stop = true;
	for (uint32_t i = 0; i < num_streams; i++)
		pthread_join(threads[i], NULL);

	for (uint32_t i = 0; i < num_streams; i++)
		bytes_received += rx[i].bytes_received;

	if (t_end == 0)
		t_end = odl_time_ns();

	{
		char size_buf2[64], tp_buf[64];
		uint64_t elapsed = t_end - t_start;

		printf("  [Server] MIMO test stopped\n");
		printf("  [Server] Received: %s in ",
		       odl_format_size(bytes_received, size_buf2,
				       sizeof(size_buf2)));

		char lat_buf[64];
		printf("%s\n", odl_format_latency(elapsed, lat_buf,
						   sizeof(lat_buf)));
		printf("  [Server] Throughput: %s\n",
		       odl_format_throughput(bytes_received, elapsed,
					    tp_buf, sizeof(tp_buf)));
	}

	/* Receive client's RESULT, send server's RESULT. */
	{
		struct odl_cli_result result;

		memset(&result, 0, sizeof(result));
		result.bytes_transferred = bytes_received;
		result.elapsed_ns = t_end - t_start;

		ret = odl_cli_recv_msg(handle, sid, msg_buf, sizeof(msg_buf),
				       &type, &seq, NULL);
		if (ret >= 0)
			odl_cli_send_msg(handle, sid, dst, ODL_CLI_MSG_RESULT, 0,
					 &result.bytes_transferred,
					 sizeof(result) - sizeof(result.hdr));
	}

	ret = 0;

out_close:
	for (uint32_t i = 0; i < opened; i++)
		odl_tb5_stream_close(handle, rx[i].sid);

	free(threads);
	free(rx);

	return ret;
}
