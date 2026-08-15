add_executable(odl_tb5_test
    odl_tb5_test_main.c
    odl_tb5_test_device.c
    odl_tb5_test_lib_api.c
    odl_tb5_test_plugin.c
)

target_include_directories(odl_tb5_test PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/include
    ${CMAKE_SOURCE_DIR}/driver/uapi
    ${CMAKE_SOURCE_DIR}/third_party/rccl
)
target_link_libraries(odl_tb5_test odl_tb5 rccl_net_odl_tb5 dl)

add_executable(odl_tb5_test_rccl_dmabuf
    odl_tb5_test_rccl_dmabuf.c
)

target_include_directories(odl_tb5_test_rccl_dmabuf PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/include
    ${CMAKE_SOURCE_DIR}/driver/uapi
)
target_link_libraries(odl_tb5_test_rccl_dmabuf odl_tb5 pthread)

add_executable(odl_tb5_pair_dmabuf
    odl_tb5_pair_dmabuf.c
)

target_include_directories(odl_tb5_pair_dmabuf PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/include
    ${CMAKE_SOURCE_DIR}/driver/uapi
)
target_link_libraries(odl_tb5_pair_dmabuf odl_tb5 pthread)

# No module, no peer, no /dev: mirrors the kernel raw_geom rule over synthetic
# page-granular SG layouts.  Fails if ODL_TB5_RAW_CELL_MAX ever stops dividing
# PAGE_SIZE (that regressed the raw gate to framed fallback + RX mismatches).
add_executable(odl_tb5_raw_geom_test
    odl_tb5_raw_geom_test.c
)

target_include_directories(odl_tb5_raw_geom_test PRIVATE
    ${CMAKE_SOURCE_DIR}/driver/uapi
)

add_executable(odl_tb5_bench_dmabuf
    odl_tb5_bench_dmabuf.c
)

target_include_directories(odl_tb5_bench_dmabuf PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/include
    ${CMAKE_SOURCE_DIR}/driver/uapi
)
target_link_libraries(odl_tb5_bench_dmabuf odl_tb5 pthread)

add_executable(odl_stream_verify
    odl_stream_verify.c
)

target_include_directories(odl_stream_verify PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/include
    ${CMAKE_SOURCE_DIR}/driver/uapi
)
target_link_libraries(odl_stream_verify odl_tb5 pthread)

add_executable(odl_tb5_test_plugin_pair
    odl_tb5_test_plugin_pair.c
)

target_include_directories(odl_tb5_test_plugin_pair PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/include
    ${CMAKE_SOURCE_DIR}/driver/uapi
    ${CMAKE_SOURCE_DIR}/third_party/rccl
)
target_link_libraries(odl_tb5_test_plugin_pair rccl_net_odl_tb5 dl)
