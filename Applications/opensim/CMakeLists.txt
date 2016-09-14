
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


# We put "-cli" in the name of the executable in the build directory; the name
# "opensim" would conflict with the "OpenSim" folder in the build directory (on
# case-insensitive filesystems). However, the executable is renamed to
# "opensim" when it is installed.
OpenSimAddApplication(NAME opensim-cli INSTALL_AS opensim
    SOURCES opensim_run_tool.h
            opensim_print_xml.h
            opensim_info.h
            opensim_update_file.h
            parse_arguments.h
    )

target_link_libraries(opensim-cli docopt)

if(BUILD_TESTING)
    subdirs(test)
endif(BUILD_TESTING)
