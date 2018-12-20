
if(MUSCOLLO_COPY_DEPENDENCIES AND APPLE)
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
        ${CMAKE_CURRENT_BINARY_DIR}/MocoInstallMacDependencyLibraries.cmake)
    configure_file(MocoInstallMacDependencyLibraries.cmake.in
            "${script}" @ONLY)
    install(SCRIPT "${script}")

endif()






include(CMakePackageConfigHelpers)

configure_package_config_file(
        MocoConfig.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/MocoConfigToInstall.cmake"
        INSTALL_DESTINATION "${MUSCOLLO_INSTALL_CMAKEDIR}"
        PATH_VARS
            CMAKE_INSTALL_PREFIX
            CMAKE_INSTALL_LIBDIR
            CMAKE_INSTALL_BINDIR
            MUSCOLLO_INSTALL_INCLUDEDIR
            MUSCOLLO_INSTALL_CMAKEDIR
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/MocoConfigToInstall.cmake"
        DESTINATION "${MUSCOLLO_INSTALL_CMAKEDIR}"
        RENAME MocoConfig.cmake)

install(FILES MocoMacros.cmake
        DESTINATION "${MUSCOLLO_INSTALL_CMAKEDIR}"
        RENAME UseMoco.cmake)

install(EXPORT MocoTargets DESTINATION "${MUSCOLLO_INSTALL_CMAKEDIR}")
