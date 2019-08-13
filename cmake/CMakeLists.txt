
if(MOCO_COPY_DEPENDENCIES AND APPLE)
    # Temporary hack to package dependencies on Macs.
    # TODO if we're building a standalone binary distribution, we should
    # use superbuild to build the dependencies.
    # TODO install(FILES
    # TODO         /usr/local/opt/llvm/lib/libc++.1.dylib
    # TODO         DESTINATION ${CMAKE_INSTALL_LIBDIR})

    # This command must be invoked from the cmake subdirectory so that the
    # editing of libtropter's link libraries is done after libtropter.dylib
    # is installed (add_subdirectory(cmake) must be after
    # add_subdirectory(Moco)).
    set(script
        ${CMAKE_CURRENT_BINARY_DIR}/OpenSimMocoInstallMacDependencyLibraries.cmake)
    configure_file(OpenSimMocoInstallMacDependencyLibraries.cmake.in
            "${script}" @ONLY)
    install(SCRIPT "${script}")

endif()






include(CMakePackageConfigHelpers)

configure_package_config_file(
        OpenSimMocoConfig.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/OpenSimMocoConfigToInstall.cmake"
        INSTALL_DESTINATION "${MOCO_INSTALL_CMAKEDIR}"
        PATH_VARS
            CMAKE_INSTALL_PREFIX
            CMAKE_INSTALL_LIBDIR
            CMAKE_INSTALL_BINDIR
            CMAKE_INSTALL_INCLUDEDIR
            MOCO_INSTALL_CMAKEDIR
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/OpenSimMocoConfigToInstall.cmake"
        DESTINATION "${MOCO_INSTALL_CMAKEDIR}"
        RENAME OpenSimMocoConfig.cmake)

install(FILES OpenSimMocoMacros.cmake
        DESTINATION "${MOCO_INSTALL_CMAKEDIR}"
        RENAME UseOpenSimMoco.cmake)

install(EXPORT OpenSimMocoTargets DESTINATION "${MOCO_INSTALL_CMAKEDIR}")
