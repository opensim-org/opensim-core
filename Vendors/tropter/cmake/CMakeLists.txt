
if(TROPTER_COPY_DEPENDENCIES AND APPLE)
    get_filename_component(gcc_libdir "${pkgcfg_lib_IPOPT_gfortran}" DIRECTORY)

    file(GLOB gfortran "${gcc_libdir}/libgfortran*.dylib")
    file(GLOB quadmath "${gcc_libdir}/libquadmath*.dylib")

    # Get the actual name of the library libgcc_s.1.dylib (can be libgcc_s.1.1.dylib),
    # or another if it changes in the future.
    execute_process(COMMAND bash "-c"
        "basename `otool -L ${IPOPT_LIBDIR}/libcoinmumps.dylib  | grep 'libgcc' | awk '{print $1}'`"
        OUTPUT_VARIABLE libgcc_name
        OUTPUT_STRIP_TRAILING_WHITESPACE)

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
            ${gcc_libdir}/${libgcc_name}

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

elseif(TROPTER_COPY_DEPENDENCIES AND UNIX)
    message(STATUS "getting dir for ${pkgcfg_lib_IPOPT_gfortran}" )
    message(STATUS "ADOLC_DIR dir ${ADOLC_DIR}" )
    message(STATUS "ColPack_ROOT_DIR dir ${ColPack_ROOT_DIR}" )
    message(STATUS "IPOPT_LIBDIR dir ${IPOPT_LIBDIR}" )
    
    get_filename_component(gcc_libdir "${pkgcfg_lib_IPOPT_gfortran}" DIRECTORY)

    file(GLOB gfortran "${gcc_libdir}/../../../x86_64-linux-gnu/libgfortran*.so*")
    file(GLOB quadmath "${gcc_libdir}/../../../x86_64-linux-gnu/libquadmath*.so*")
    message(STATUS "gfortran list ${gfortran}" )
    message(STATUS "quadmath list ${quadmath}" )

    install(FILES
            ${ADOLC_DIR}/lib64/libadolc.so.2
            ${ADOLC_DIR}/lib64/libadolc.so.2.1.0
            ${ADOLC_DIR}/lib64/libadolc.so
            # /usr/local/opt/boost/lib/libboost_system.so
            ${ColPack_ROOT_DIR}/lib/libColPack.so.0
            ${ColPack_ROOT_DIR}/lib/libColPack.so
            ${IPOPT_LIBDIR}/libipopt.so.1.10.8
            ${IPOPT_LIBDIR}/libipopt.so.1
            ${IPOPT_LIBDIR}/libipopt.so
            ${IPOPT_LIBDIR}/libcoinmumps.so.1.6.0
            ${IPOPT_LIBDIR}/libcoinmumps.so.1
            ${IPOPT_LIBDIR}/libcoinmumps.so
            ${IPOPT_LIBDIR}/libcoinmetis.so.1.3.5
            ${IPOPT_LIBDIR}/libcoinmetis.so.1
            ${IPOPT_LIBDIR}/libcoinmetis.so

            ${gfortran}
            ${quadmath}
            ${gcc_libdir}/libgcc_s.so

            DESTINATION ${CMAKE_INSTALL_LIBDIR})
        set(script
            ${CMAKE_CURRENT_BINARY_DIR}/tropter_install_linux_dependency_libraries.cmake)
        configure_file(tropter_install_linux_dependency_libraries.cmake.in
                   "${script}" @ONLY)
        install(SCRIPT "${script}")
endif()


