/*
 * OdinLink — Test: RCCL/NCCL Plugin Interface
 *
 * Loads the RCCL net v7 plugin (librccl_net_odl_tb5.so) and verifies
 * that the plugin entry points (open, close, send, recv, etc.) work
 * correctly. This is the path AMD GPUs use for collective operations
 * over Thunderbolt.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <dlfcn.h>

#include "net_v7.h"

static int test_count;
static int pass_count;
static int fail_count;

#define TEST(name) do { \
	test_count++; \
	printf("  [TEST] %s... ", name); \
} while (0)

#define PASS() do { pass_count++; printf("PASS\n"); } while (0)
#define FAIL(msg) do { fail_count++; printf("FAIL: %s\n", msg); } while (0)

/* Import plugin symbol directly (linked against librccl_net_odl_tb5.so) */
extern rcclNet_v7_t rcclNetPlugin_v7;

int odl_tb5_test_plugin(void)
{
	rcclNet_v7_t *plugin = &rcclNetPlugin_v7;
	int ret;

	printf("\n--- RCCL Plugin Tests ---\n");

	/* Test: Plugin name */
	TEST("Plugin name is ODL_TB5");
	if (plugin->name && strcmp(plugin->name, "ODL_TB5") == 0) {
		PASS();
	} else {
		FAIL(plugin->name ? plugin->name : "NULL");
	}

	/* Test: All function pointers are set */
	TEST("All function pointers non-NULL");
	if (plugin->init && plugin->devices && plugin->getProperties &&
	    plugin->listen && plugin->connect && plugin->accept &&
	    plugin->regMr && plugin->regMrDmaBuf && plugin->deregMr &&
	    plugin->closeListen && plugin->isend && plugin->irecv &&
	    plugin->iflush && plugin->test && plugin->closeSend &&
	    plugin->closeRecv && plugin->getDeviceMr &&
	    plugin->irecvConsumed) {
		PASS();
	} else {
		FAIL("Some function pointers are NULL");
	}

	/* Test: Initialize plugin */
	TEST("plugin->init()");
	{
		rcclResult_t res = plugin->init(NULL);
		if (res == rcclSuccess) {
			PASS();
		} else {
			FAIL("init returned error");
		}
	}

	/* DMA-BUF registrations are real: the plugin duplicates the fd and
	 * keeps it with the mhandle for the transfer path. */
	TEST("regMrDmaBuf accepts and retains the fd");
	{
		int fd = open("/dev/null", O_RDONLY);
		void *mhandle = NULL;
		rcclResult_t res;

		if (fd < 0) {
			FAIL("cannot open /dev/null");
		} else {
			res = plugin->regMrDmaBuf(NULL, (void *)0x1000, 4096,
						  NCCL_PTR_DMABUF, 0, fd,
						  &mhandle);
			if (res == rcclSuccess && mhandle != NULL) {
				rcclResult_t dres = plugin->deregMr(NULL,
								    mhandle);
				if (dres == rcclSuccess)
					PASS();
				else
					FAIL("deregMr failed after regMrDmaBuf");
			} else {
				FAIL("expected rcclSuccess and a non-NULL handle");
			}
			close(fd);
		}
	}

	/* A closed fd cannot be registered — must fail loudly, not claim
	 * success while dropping the fd. */
	TEST("regMrDmaBuf rejects an invalid fd");
	{
		int fd = open("/dev/null", O_RDONLY);
		void *mhandle = (void *)1;
		rcclResult_t res;

		if (fd < 0) {
			FAIL("cannot open /dev/null");
		} else {
			close(fd);   /* fd now invalid */
			res = plugin->regMrDmaBuf(NULL, NULL, 4096,
						  NCCL_PTR_DMABUF, 128, fd,
						  &mhandle);
			if (res != rcclSuccess && mhandle == NULL)
				PASS();
			else
				FAIL("expected an error and a NULL handle");
		}
	}

	/* Test: Enumerate devices */
	TEST("plugin->devices()");
	{
		int ndev = 0;
		rcclResult_t res = plugin->devices(&ndev);
		if (res == rcclSuccess) {
			printf("PASS (found %d devices)\n", ndev);
			pass_count++;
		} else {
			FAIL("devices() returned error");
		}
	}

	/* Test: Get properties for device 0 (if available) */
	TEST("plugin->getProperties(0)");
	{
		int ndev = 0;
		plugin->devices(&ndev);

		if (ndev > 0) {
			rcclNetProperties_v7_t props;
			int want = NCCL_PTR_HOST | NCCL_PTR_CUDA |
				   NCCL_PTR_DMABUF;
			rcclResult_t res = plugin->getProperties(0, &props);
			if (res == rcclSuccess && props.speed > 0 &&
			    props.ptrSupport == want) {
				printf("PASS (name=%s, speed=%d, ptr=%d)\n",
				       props.name, props.speed, props.ptrSupport);
				pass_count++;
			} else {
				FAIL("getProperties returned invalid host properties");
			}
		} else {
			printf("SKIP (no devices)\n");
			pass_count++;
		}
	}

	/* Test: Listen on device 0 */
	TEST("plugin->listen(0)");
	{
		int ndev = 0;
		plugin->devices(&ndev);

		if (ndev > 0) {
			void *listenComm = NULL;
			char handle[64] = {0};
			rcclResult_t res = plugin->listen(0, handle, &listenComm);
			if (res == rcclSuccess && listenComm != NULL) {
				PASS();
				plugin->closeListen(listenComm);
			} else {
				FAIL("listen returned error");
			}
		} else {
			printf("SKIP (no devices)\n");
			pass_count++;
		}
	}

	/* Test: Invalid device index */
	TEST("getProperties(-1) returns error");
	{
		rcclNetProperties_v7_t props;
		rcclResult_t res = plugin->getProperties(-1, &props);
		if (res == rcclInvalidArgument) {
			PASS();
		} else {
			FAIL("Expected rcclInvalidArgument");
		}
	}

	return fail_count;
}
