
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
    # add_subdirectory(Muscollo)).
    install(SCRIPT MuscolloInstallMacDependencyLibraries.cmake)

endif()






include(CMakePackageConfigHelpers)

configure_package_config_file(
        MuscolloConfig.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/MuscolloConfigToInstall.cmake"
        INSTALL_DESTINATION "${MUSCOLLO_INSTALL_CMAKEDIR}"
        PATH_VARS
            CMAKE_INSTALL_PREFIX
            CMAKE_INSTALL_LIBDIR
            CMAKE_INSTALL_BINDIR
            MUSCOLLO_INSTALL_INCLUDEDIR
            MUSCOLLO_INSTALL_CMAKEDIR
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/MuscolloConfigToInstall.cmake"
        DESTINATION "${MUSCOLLO_INSTALL_CMAKEDIR}"
        RENAME MuscolloConfig.cmake)

install(FILES MuscolloMacros.cmake
        DESTINATION "${MUSCOLLO_INSTALL_CMAKEDIR}"
        RENAME UseMuscollo.cmake)

install(EXPORT MuscolloTargets DESTINATION "${MUSCOLLO_INSTALL_CMAKEDIR}")
