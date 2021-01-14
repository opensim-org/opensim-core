
if(TROPTER_COPY_DEPENDENCIES AND APPLE)
    get_filename_component(gcc_libdir "${pkgcfg_lib_IPOPT_gfortran}" DIRECTORY)

    file(GLOB gfortran "${gcc_libdir}/libgfortran*.dylib")
    file(GLOB quadmath "${gcc_libdir}/libquadmath*.dylib")

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

            ${gfortran}
            ${quadmath}
            ${gcc_libdir}/libgcc_s.1.dylib

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
if(TROPTER_COPY_DEPENDENCIES AND UNIX)
    get_filename_component(gcc_libdir "${pkgcfg_lib_IPOPT_gfortran}" DIRECTORY)

    file(GLOB gfortran "${gcc_libdir}/libgfortran*.so")
    file(GLOB quadmath "${gcc_libdir}/libquadmath*.so")

    install(FILES
            ${ADOLC_DIR}/lib64/libadolc.2.so
            ${ADOLC_DIR}/lib64/libadolc.so
            # /usr/local/opt/boost/lib/libboost_system.so
            ${ColPack_ROOT_DIR}/lib/libColPack.0.so
            ${IPOPT_LIBDIR}/libipopt.1.10.8.so
            ${IPOPT_LIBDIR}/libipopt.1.so
            ${IPOPT_LIBDIR}/libipopt.so
            ${IPOPT_LIBDIR}/libcoinmumps.1.6.0.so
            ${IPOPT_LIBDIR}/libcoinmumps.1.so
            ${IPOPT_LIBDIR}/libcoinmumps.so
            ${IPOPT_LIBDIR}/libcoinmetis.1.3.5.so
            ${IPOPT_LIBDIR}/libcoinmetis.1.so
            ${IPOPT_LIBDIR}/libcoinmetis.so

            ${gfortran}
            ${quadmath}
            ${gcc_libdir}/libgcc_s.1.so

            DESTINATION ${CMAKE_INSTALL_LIBDIR})

    # This command must be invoked from the cmake subdirectory so that the
    # editing of libtropter's link libraries is done after libtropter.so
    # is installed (add_subdirectory(cmake) must be after
    # add_subdirectory(tropter)).
    ## set(script
    ## ${CMAKE_CURRENT_BINARY_DIR}/tropter_install_mac_dependency_libraries.cmake)
    ## configure_file(tropter_install_mac_dependency_libraries.cmake.in
    ##        "${script}" @ONLY)
    ## install(SCRIPT "${script}")

endif()



