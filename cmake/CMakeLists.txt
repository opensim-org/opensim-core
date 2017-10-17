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
