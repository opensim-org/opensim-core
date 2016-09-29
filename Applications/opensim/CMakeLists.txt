
# Docopt *is* required, but we omit the "REQUIRED" flag for find_package()
# since we want to give a custom error message. If docopt is not found, then
# CMake will still produce a warning telling users to set CMAKE_PREFIX_PATH or
# docopt_DIR.
find_package(docopt 0.6.1
             HINTS "${OPENSIM_DEPENDENCIES_DIR}/docopt/lib/cmake/docopt")
if(WIN32)
    # Back out the directory containing docopt.dll
    # The docopt-config.cmake file does not define a variable containing
    # the docopt installation directory.
    # The location of docopt-config.cmake within the docopt installation is
    # lib/cmake/docopt.
    # docopt_DIR is the path to the docopt-config.cmake file.
    # So we remove the last two directories from docopt_DIR.
    get_filename_component(docopt_lib_dir ${docopt_DIR} PATH)
    get_filename_component(docopt_lib_dir ${docopt_lib_dir} PATH)
    CopyDependencyDLLsForWin(docopt ${docopt_lib_dir})
endif()

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
