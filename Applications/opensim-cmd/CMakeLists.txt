OpenSimAddApplication(NAME opensim-cmd
    SOURCES opensim-cmd_run-tool.h
            opensim-cmd_print-xml.h
            opensim-cmd_info.h
            opensim-cmd_update-file.h
            parse_arguments.h
    )

target_link_libraries(opensim-cmd osimdocopt osimMoco osimJAM hdf5-static hdf5_cpp-static hdf5_hl-static)

if(BUILD_TESTING)
    subdirs(test)
endif(BUILD_TESTING)
