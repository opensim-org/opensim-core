
if(TROPTER_COPY_DEPENDENCIES AND APPLE)
    get_filename_component(gcc_libdir "${pkgcfg_lib_IPOPT_gfortran}" DIRECTORY)

    file(GLOB gfortran "${gcc_libdir}/libgfortran*.dylib")
    file(GLOB quadmath "${gcc_libdir}/libquadmath*.dylib")

    # Check for arm64 architecture
    execute_process(
        COMMAND uname -m
        OUTPUT_VARIABLE MACHINE_ARCH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        )

    # For ARM-based Macs, install a different set of libraries (without mumps 
    # and metis) because of the different way dependencies are built.
    if(MACHINE_ARCH STREQUAL "arm64")
        install(FILES
                ${ADOLC_DIR}/lib64/libadolc.2.dylib
                ${ADOLC_DIR}/lib64/libadolc.dylib
                ${ColPack_ROOT_DIR}/lib/libColPack.0.dylib
                ${IPOPT_LIBDIR}/libipopt.1.10.8.dylib
                ${IPOPT_LIBDIR}/libipopt.1.dylib
                ${IPOPT_LIBDIR}/libipopt.dylib

                ${gfortran}
                ${quadmath}
                ${gcc_libdir}/libgcc_s.1.dylib

                DESTINATION ${CMAKE_INSTALL_LIBDIR})

    # For x64 Macs, make sure to also include mumps and metis libraries.
    else()
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
    endif()

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
        get_filename_component(gcc_libdir "${pkgcfg_lib_IPOPT_gfortran}" DIRECTORY)

        file(GLOB gfortran "${gcc_libdir}/libgfortran*.so")
        file(GLOB quadmath "${gcc_libdir}/libquadmath*.so")

        install(FILES
           ${ADOLC_DIR}/lib64/libadolc.so.2.1.0 DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()



