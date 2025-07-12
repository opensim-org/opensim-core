OpenSimAddApplication(NAME opensim-cmd
    SOURCES opensim-cmd_run-tool.h
            opensim-cmd_print-xml.h
            opensim-cmd_info.h
            opensim-cmd_update-file.h
            parse_arguments.h
    )

<<<<<<< HEAD
target_link_libraries(opensim-cmd docopt_s osimMoco osimJAM hdf5-static hdf5_cpp-static hdf5_hl-static)
=======
target_link_libraries(opensim-cmd osimdocopt osimMoco)
>>>>>>> 74e1b2ed3091c87ab0b273e8d1c2b8f0f41b85aa

if(BUILD_TESTING)
    subdirs(test)
endif(BUILD_TESTING)
