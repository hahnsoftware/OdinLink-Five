/* SPDX-License-Identifier: MIT */

#define _GNU_SOURCE
#include <dlfcn.h>
#include <infiniband/verbs.h>
#include <stdio.h>
#include <string.h>

typedef struct ibv_device **(*get_device_list_fn)(int *);
typedef void (*free_device_list_fn)(struct ibv_device **);

struct required_symbol {
    const char *name;
    const char *version;
};

static const struct required_symbol required[] = {
    {"ibv_ack_async_event", "IBVERBS_1.1"},
    {"ibv_alloc_pd", "IBVERBS_1.1"},
    {"ibv_close_device", "IBVERBS_1.1"},
    {"ibv_create_cq", "IBVERBS_1.1"},
    {"ibv_create_qp", "IBVERBS_1.1"},
    {"ibv_dealloc_pd", "IBVERBS_1.1"},
    {"ibv_dereg_mr", "IBVERBS_1.1"},
    {"ibv_destroy_cq", "IBVERBS_1.1"},
    {"ibv_destroy_qp", "IBVERBS_1.1"},
    {"ibv_event_type_str", "IBVERBS_1.1"},
    {"ibv_fork_init", "IBVERBS_1.1"},
    {"ibv_free_device_list", "IBVERBS_1.1"},
    {"ibv_get_async_event", "IBVERBS_1.1"},
    {"ibv_get_device_list", "IBVERBS_1.1"},
    {"ibv_get_device_name", "IBVERBS_1.1"},
    {"ibv_modify_qp", "IBVERBS_1.1"},
    {"ibv_open_device", "IBVERBS_1.1"},
    {"ibv_poll_cq", "IBVERBS_1.1"},
    {"ibv_post_recv", "IBVERBS_1.1"},
    {"ibv_post_send", "IBVERBS_1.1"},
    {"ibv_query_device", "IBVERBS_1.1"},
    {"ibv_query_gid", "IBVERBS_1.1"},
    {"ibv_query_port", "IBVERBS_1.1"},
    {"ibv_query_qp", "IBVERBS_1.1"},
    {"ibv_reg_mr", "IBVERBS_1.1"},
    {"ibv_reg_mr_iova2", "IBVERBS_1.8"},
    {"ibv_query_ece", "IBVERBS_1.10"},
    {"ibv_set_ece", "IBVERBS_1.10"},
    {"_ibv_query_gid_ex", "IBVERBS_1.11"},
    {"ibv_reg_dmabuf_mr", "IBVERBS_1.12"},
};

int main(void)
{
    get_device_list_fn get_device_list;
    free_device_list_fn free_device_list;
    struct ibv_device **list;
    void *handle;
    int count = 0;
    int found = 0;

    handle = dlopen("libibverbs.so.1", RTLD_NOW | RTLD_LOCAL);
    if (!handle) {
        fprintf(stderr, "FAIL: dlopen libibverbs: %s\n", dlerror());
        return 1;
    }
    for (size_t i = 0; i < sizeof(required) / sizeof(required[0]); i++) {
        if (!dlvsym(handle, required[i].name, required[i].version)) {
            fprintf(stderr, "FAIL: missing %s@%s: %s\n", required[i].name,
                    required[i].version, dlerror());
            dlclose(handle);
            return 1;
        }
    }

    get_device_list = (get_device_list_fn)dlvsym(
        handle, "ibv_get_device_list", "IBVERBS_1.1");
    free_device_list = (free_device_list_fn)dlvsym(
        handle, "ibv_free_device_list", "IBVERBS_1.1");
    list = get_device_list(&count);
    for (int i = 0; list && i < count; i++) {
        if (strcmp(list[i]->name, "odl_tb5_0") == 0)
            found = 1;
    }
    if (list)
        free_device_list(list);
    dlclose(handle);

    if (!found) {
        fprintf(stderr, "FAIL: private libibverbs handle cannot see odl_tb5_0\n");
        return 1;
    }
    puts("PASS: RCCL private handle resolves all verbs and sees odl_tb5_0");
    return 0;
}
