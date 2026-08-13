add_library(rccl_net_odl_tb5 SHARED
    src/odl_tb5_plugin.c
)

target_include_directories(rccl_net_odl_tb5 PRIVATE
    ${CMAKE_SOURCE_DIR}/third_party/rccl
    ${CMAKE_SOURCE_DIR}/lib/include
)
target_link_libraries(rccl_net_odl_tb5 odl_tb5)

# Keep the canonical library name, but let the installed plugin find its
# companion userspace library without requiring LD_LIBRARY_PATH.
set_target_properties(rccl_net_odl_tb5 PROPERTIES
    INSTALL_RPATH "$ORIGIN"
)

include(GNUInstallDirs)
install(TARGETS rccl_net_odl_tb5
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    COMPONENT rccl
)

# RCCL probes these two names, not the target's canonical
# librccl_net_odl_tb5.so filename.  Install aliases instead of duplicating the
# plugin binary.  DESTDIR is honored so this also works for CPack packages.
set(_RCCL_PLUGIN_FILENAME
    "${CMAKE_SHARED_LIBRARY_PREFIX}rccl_net_odl_tb5${CMAKE_SHARED_LIBRARY_SUFFIX}")
install(CODE "
	set(_install_libdir \"${CMAKE_INSTALL_LIBDIR}\")
	if(IS_ABSOLUTE \"\${_install_libdir}\")
		set(_alias_dir \"\$ENV{DESTDIR}\${_install_libdir}\")
	else()
		set(_alias_dir
			\"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/\${_install_libdir}\")
	endif()
    foreach(_alias IN ITEMS librccl-net.so librccl-net-ODL_TB5.so)
		set(_alias_path \"\${_alias_dir}/\${_alias}\")
        file(REMOVE \"\${_alias_path}\")
        execute_process(
            COMMAND \"${CMAKE_COMMAND}\" -E create_symlink
                    \"${_RCCL_PLUGIN_FILENAME}\" \"\${_alias_path}\"
            RESULT_VARIABLE _alias_result
        )
        if(NOT _alias_result EQUAL 0)
            message(FATAL_ERROR \"Failed to install RCCL plugin alias \${_alias}\")
        endif()
    endforeach()
" COMPONENT rccl)
