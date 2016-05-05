
# We put "-cli" in the name of the executable in the build directory; "opensim"
# since it conflicts with the "OpenSim" folder in the build directory (on
# case-insensitive filesystems). However, the executable is renamed to
# "opensim" when it is installed.
OpenSimAddApplication(NAME opensim-cli INSTALL_AS opensim
    SOURCES opensim_run_tool.h
            opensim_print_xml.h
            opensim_info.h
            opensim_update_file.h
            parse_arguments.h
            docopt/docopt.h
            docopt/docopt.cpp
            docopt/docopt_private.h
            docopt/docopt_util.h
            docopt/docopt_value.h
    )

# Include the docopt.cpp directory.
include_directories(docopt)

if(BUILD_TESTING)
    subdirs(test)
endif(BUILD_TESTING)
