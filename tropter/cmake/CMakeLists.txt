
if(TROPTER_COPY_DEPENDENCIES AND APPLE)
    # Temporary hack to package dependencies on Macs.
    # TODO if we're building a standalone binary distribution, we should
    # use superbuild to build the dependencies.
    install(FILES
            ${ADOLC_DIR}/lib64/libadolc.2.dylib
            ${ADOLC_DIR}/lib64/libadolc.dylib
            # /usr/local/opt/boost/lib/libboost_system.dylib
            ${ColPack_ROOT_DIR}/lib/libColPack.0.dylib
            ${IPOPT_LIBDIR}/libipopt.1.10.8.dylib
            ${IPOPT_LIBDIR}/libipopt.1.dylib
            ${IPOPT_LIBDIR}/libipopt.dylib
            ${IPOPT_LIBDIR}/libcoinmumps.1.6.0.dylib
            ${IPOPT_LIBDIR}/libcoinmumps.1.dylib
            ${IPOPT_LIBDIR}/libcoinmumps.dylib
            ${IPOPT_LIBDIR}/libcoinmetis.1.3.5.dylib
            ${IPOPT_LIBDIR}/libcoinmetis.1.dylib
            ${IPOPT_LIBDIR}/libcoinmetis.dylib

            # From Homebrew:
            /usr/local/opt/gcc/lib/gcc/8/libgfortran.5.dylib
            /usr/local/lib/gcc/7/libgcc_s.1.dylib
            /usr/local/opt/gcc/lib/gcc/8/libquadmath.0.dylib

            # TODO: Parse and use
            # ${pkgcfg_lib_IPOPT_gfortran}
            # ${pkgcfg_lib_IPOPT_quadmath}
            # ${pkgcfg_lib_IPOPT_coinmetis}
            # ${pkgcfg_lib_IPOPT_coinmumps}

            DESTINATION ${CMAKE_INSTALL_LIBDIR})

    # This command must be invoked from the cmake subdirectory so that the
    # editing of libtropter's link libraries is done after libtropter.dylib
    # is installed (add_subdirectory(cmake) must be after
    # add_subdirectory(tropter)).
    set(script
     ${CMAKE_CURRENT_BINARY_DIR}/tropter_install_mac_dependency_libraries.cmake)
    configure_file(tropter_install_mac_dependency_libraries.cmake.in
            "${script}" @ONLY)
    install(SCRIPT "${script}")

endif()



