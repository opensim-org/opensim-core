
if(TROPTER_COPY_DEPENDENCIES AND APPLE)
    # Temporary hack to package dependencies on Macs.
    # TODO if we're building a standalone binary distribution, we should
    # use superbuild to build the dependencies.
    install(FILES
            /usr/local/opt/adol-c/lib/libadolc.2.dylib
            /usr/local/opt/boost/lib/libboost_system.dylib
            /usr/local/opt/colpack/lib/libColPack.0.dylib

            /usr/local/opt/ipopt/lib/libipopt.1.dylib
            /usr/local/opt/ipopt/lib/libipopt.1.10.8.dylib
            /usr/local/opt/mumps/lib/libdmumps.dylib
            /usr/local/opt/gcc/lib/gcc/7/libgfortran.4.dylib
            /usr/local/lib/gcc/7/libgcc_s.1.dylib
            /usr/local/opt/gcc/lib/gcc/7/libquadmath.0.dylib

            /usr/local/opt/mumps/lib/libmumps_common.dylib
            /usr/local/opt/mumps/lib/libpord.dylib
            /usr/local/opt/mumps/lib/libmpiseq.dylib
            DESTINATION ${CMAKE_INSTALL_LIBDIR})

    # This command must be invoked from the cmake subdirectory so that the
    # editing of libtropter's link libraries is done after libtropter.dylib
    # is installed (add_subdirectory(cmake) must be after
    # add_subdirectory(tropter)).
    install(SCRIPT tropter_install_mac_dependency_libraries.cmake)

endif()



