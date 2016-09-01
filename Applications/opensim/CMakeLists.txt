
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
