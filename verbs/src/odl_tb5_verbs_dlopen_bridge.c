/* SPDX-License-Identifier: MIT
 *
 * RCCL loads libibverbs through a private dlopen() handle and then resolves
 * verbs calls from that handle. That skips normal LD_PRELOAD interposition.
 * Redirect only RCCL's libibverbs load to the standalone OdinLink provider;
 * calls the provider does internally still reach the system libibverbs.
 */

#define _GNU_SOURCE
#include <dlfcn.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

typedef void *(*dlopen_fn)(const char *, int);

static dlopen_fn real_dlopen;
static pthread_once_t resolve_once = PTHREAD_ONCE_INIT;

static void resolve_dlopen(void)
{
    real_dlopen = (dlopen_fn)dlsym(RTLD_NEXT, "dlopen");
}

static bool is_libibverbs(const char *filename)
{
    const char *base;

    if (!filename)
        return false;
    base = strrchr(filename, '/');
    base = base ? base + 1 : filename;
    return strcmp(base, "libibverbs.so") == 0 ||
           strcmp(base, "libibverbs.so.1") == 0;
}

static bool caller_is_rccl(void *return_address)
{
    Dl_info caller = {0};
    const char *all = getenv("ODL_TB5_VERBS_DLOPEN_BRIDGE_ALL");

    if (all && strcmp(all, "0") != 0)
        return true;
    if (!dladdr(return_address, &caller) || !caller.dli_fname)
        return false;
    return strstr(caller.dli_fname, "librccl.so") != NULL ||
           strstr(caller.dli_fname, "libnccl.so") != NULL;
}

void *dlopen(const char *filename, int flags)
{
    const char *provider;
    void *handle;

    pthread_once(&resolve_once, resolve_dlopen);
    if (!real_dlopen)
        return NULL;

    if (!is_libibverbs(filename) ||
        !caller_is_rccl(__builtin_return_address(0)))
        return real_dlopen(filename, flags);

    provider = getenv("ODL_TB5_VERBS_LIBRARY");
    if (!provider || !provider[0])
        provider = "libodl_tb5_verbs.so.0";

    handle = real_dlopen(provider, flags);
    if (handle)
        return handle;

    /* Preserve normal RCCL behavior if the OdinLink library is absent. */
    return real_dlopen(filename, flags);
}
