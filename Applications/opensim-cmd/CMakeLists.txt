
# Docopt *is* required, but we omit the "REQUIRED" flag for find_package()
# since we want to give a custom error message. If docopt is not found, then
# CMake will still produce a warning telling users to set CMAKE_PREFIX_PATH or
# docopt_DIR.
find_package(docopt 0.6.1
             HINTS "${OPENSIM_DEPENDENCIES_DIR}/docopt/lib/cmake/docopt")

if(NOT docopt_FOUND)
    message(FATAL_ERROR "Dependency 'docopt' not found, but is needed for \
            command-line argument parsing; see the related CMake Warning.\
            Follow the Superbuild instructions in README.md to obtain docopt.")
endif()

OpenSimAddApplication(NAME opensim-cmd
    SOURCES opensim-cmd_run-tool.h
            opensim-cmd_print-xml.h
            opensim-cmd_info.h
            opensim-cmd_update-file.h
            parse_arguments.h
    )

target_link_libraries(opensim-cmd docopt_s osimMoco)

if(BUILD_TESTING)
    subdirs(test)
endif(BUILD_TESTING)
