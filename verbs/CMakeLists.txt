# OdinLink Verbs Provider — libibverbs-compatible RDMA provider
#
# Build targets:
#   1. libodl_tb5_verbs.so         — Standalone (symbol interposition)
#   2. libodl_tb5-rdmav34.so       — rdma-core provider plugin
#
# Dependencies: libodl_tb5, libibverbs (rdma-core), pthreads

cmake_minimum_required(VERSION 3.10)

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(IBVERBS QUIET libibverbs)
endif()

if(NOT IBVERBS_FOUND)
    message(STATUS "libibverbs not found — verbs provider disabled")
    return()
endif()

# Sources shared between standalone lib and provider plugin
set(VERBS_CORE_SOURCES
    src/odl_tb5_verbs_device.c
    src/odl_tb5_verbs_pd.c
    src/odl_tb5_verbs_mr.c
    src/odl_tb5_verbs_cq.c
    src/odl_tb5_verbs_qp.c
    src/odl_tb5_verbs_ops.c
)

# Common include dirs
set(VERBS_PRIVATE_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/lib/include
    ${IBVERBS_INCLUDE_DIRS}
)

# Common link libs
set(VERBS_LIBS
    odl_tb5
    ${IBVERBS_LIBRARIES}
    pthread
    dl
    m
)

# ═══════════════════════════════════════════════════════════════════════
# 1. Standalone library — symbol interposition via ibv_open_device
# ═══════════════════════════════════════════════════════════════════════

add_library(odl_tb5_verbs SHARED
    ${VERBS_CORE_SOURCES}
    src/odl_tb5_verbs_main.c
)

target_include_directories(odl_tb5_verbs PRIVATE ${VERBS_PRIVATE_INCLUDES})
target_link_libraries(odl_tb5_verbs ${VERBS_LIBS})
set_property(TARGET odl_tb5_verbs APPEND_STRING PROPERTY LINK_FLAGS
    " -Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/src/odl_tb5_verbs.map")
set_property(TARGET odl_tb5_verbs APPEND PROPERTY LINK_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/odl_tb5_verbs.map)

set_target_properties(odl_tb5_verbs PROPERTIES
    VERSION    0.1.0
    SOVERSION  0
    INSTALL_RPATH "${CMAKE_INSTALL_LIBDIR}"
    BUILD_WITH_INSTALL_RPATH ON
)

install(TARGETS odl_tb5_verbs
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    COMPONENT verbs
)

# RCCL/NCCL resolve verbs from a private dlopen() handle, which bypasses
# ordinary symbol preloading. This bridge redirects that one load to
# the standalone OdinLink provider while keeping the standard NET/IB path.
add_library(odl_tb5_verbs_dlopen_bridge SHARED
    src/odl_tb5_verbs_dlopen_bridge.c)
target_link_libraries(odl_tb5_verbs_dlopen_bridge dl pthread)
set_target_properties(odl_tb5_verbs_dlopen_bridge PROPERTIES
    VERSION 0.1.0
    SOVERSION 0
    INSTALL_RPATH "${CMAKE_INSTALL_LIBDIR}"
    BUILD_WITH_INSTALL_RPATH ON)
install(TARGETS odl_tb5_verbs_dlopen_bridge
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    COMPONENT verbs)

# ═══════════════════════════════════════════════════════════════════════
# 2. rdma-core provider plugin — registered with libibverbs
# ═══════════════════════════════════════════════════════════════════════

option(ODL_VERBS_BUILD_PROVIDER_PLUGIN "Build rdma-core provider plugin" ON)

if(ODL_VERBS_BUILD_PROVIDER_PLUGIN)
    # Determine provider plugin install directory
    unset(IBVERBS_PROVIDER_DIR)
    foreach(DIR
        "/usr/lib/aarch64-linux-gnu/libibverbs"
        "/usr/lib/x86_64-linux-gnu/libibverbs"
        "/usr/lib64/libibverbs"
        "/usr/lib/libibverbs"
        "${CMAKE_INSTALL_LIBDIR}/libibverbs")
        if(IS_DIRECTORY "${DIR}")
            set(IBVERBS_PROVIDER_DIR "${DIR}")
            break()
        endif()
    endforeach()
    if(NOT IBVERBS_PROVIDER_DIR)
        set(IBVERBS_PROVIDER_DIR "${CMAKE_INSTALL_LIBDIR}/libibverbs")
    endif()

    add_library(odl_tb5_verbs_provider SHARED
        ${VERBS_CORE_SOURCES}
        src/odl_tb5_provider_plugin.c
    )

    target_include_directories(odl_tb5_verbs_provider PRIVATE
        ${VERBS_PRIVATE_INCLUDES}
    )

    target_link_libraries(odl_tb5_verbs_provider
        ${VERBS_LIBS}
    )

    set_target_properties(odl_tb5_verbs_provider PROPERTIES
        LIBRARY_OUTPUT_NAME "odl_tb5-rdmav34"
        VERSION    0.1.0
        SOVERSION  34
        INSTALL_RPATH "${CMAKE_INSTALL_LIBDIR}"
        BUILD_WITH_INSTALL_RPATH ON
    )

    install(TARGETS odl_tb5_verbs_provider
        LIBRARY DESTINATION ${IBVERBS_PROVIDER_DIR}
        COMPONENT verbs
    )

    message(STATUS "  Provider plugin dir: ${IBVERBS_PROVIDER_DIR}")
endif()

# ═══════════════════════════════════════════════════════════════════════
# Public header
# ═══════════════════════════════════════════════════════════════════════

install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/odl_tb5_verbs_wrapper.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/odl_tb5
    COMPONENT library-dev
)

message(STATUS "BUILD_VERBS: ON")
message(STATUS "  Standalone lib:   odl_tb5_verbs")
message(STATUS "  Provider plugin:  ${ODL_VERBS_BUILD_PROVIDER_PLUGIN}")

# Hardware-facing smoke tests. They build with the provider but are not
# registered with CTest because they require connected hosts and the driver.
foreach(TEST_NAME
    test_verbs_basic
    test_verbs_dmabuf
    test_verbs_write_imm
    bench_verbs_dmabuf
    test_verbs_rccl_probe)
    add_executable(${TEST_NAME} tests/${TEST_NAME}.c)
    target_include_directories(${TEST_NAME} PRIVATE ${VERBS_PRIVATE_INCLUDES})
    target_link_libraries(${TEST_NAME} odl_tb5_verbs ${VERBS_LIBS})
    set_target_properties(${TEST_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/tests)
endforeach()

add_executable(test_verbs_dlopen_probe tests/test_verbs_dlopen_probe.c)
target_include_directories(test_verbs_dlopen_probe PRIVATE ${IBVERBS_INCLUDE_DIRS})
target_link_libraries(test_verbs_dlopen_probe dl)
set_target_properties(test_verbs_dlopen_probe PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/tests)
