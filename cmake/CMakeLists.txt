# CMake packaging so people can use OpenSim.
# ------------------------------------------

# Requires CMake 2.8.8.
include(CMakePackageConfigHelpers)

# The generated file must not be named OpenSimConfig.cmake, otherwise CMake
# will pick it up and think that the build tree contains an OpenSim
# installation.
configure_package_config_file(
    OpenSimConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfigToInstall.cmake"
    INSTALL_DESTINATION "${OPENSIM_INSTALL_CMAKEDIR}"
    PATH_VARS # Variables to edit in OpenSimConfig.cmake.in.
        CMAKE_INSTALL_PREFIX
        CMAKE_INSTALL_INCLUDEDIR
        CMAKE_INSTALL_BINDIR
        CMAKE_INSTALL_LIBDIR
        OPENSIM_INSTALL_CMAKEDIR
        OPENSIM_INSTALL_SIMBODYDIR
        OPENSIM_INSTALL_SPDLOGDIR
    )

# Version file.
write_basic_config_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfigVersion.cmake"
    VERSION "${OPENSIM_RELEASE_VERSION}"
    COMPATIBILITY SameMajorVersion)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfigToInstall.cmake"
    DESTINATION "${OPENSIM_INSTALL_CMAKEDIR}"
    RENAME OpenSimConfig.cmake
    )

install(
    FILES
        "SampleCMakeLists.txt"
        "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfigVersion.cmake"
    DESTINATION
        "${OPENSIM_INSTALL_CMAKEDIR}"
        )

install(FILES OpenSimMacros.cmake
    DESTINATION "${OPENSIM_INSTALL_CMAKEDIR}"
    RENAME UseOpenSim.cmake)

install(EXPORT OpenSimTargets DESTINATION "${OPENSIM_INSTALL_CMAKEDIR}")

if(APPLE AND OPENSIM_COPY_DEPENDENCIES AND (NOT (DEFINED OPENSIM_WITH_CASADI AND NOT ${OPENSIM_WITH_CASADI})))
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
    if (OPENSIM_WITH_CASADI)
        get_target_property(CASADI_LIBRARY_LOCATION casadi LOCATION)
        get_filename_component(CASADI_LIBDIR "${CASADI_LIBRARY_LOCATION}" DIRECTORY)
    endif()
    set(script
        ${CMAKE_CURRENT_BINARY_DIR}/OpenSimMocoInstallMacDependencyLibraries.cmake)
    configure_file(OpenSimMocoInstallMacDependencyLibraries.cmake.in
            "${script}" @ONLY)
    # Variables are not forwarded to the install script, so add a line
    # to set the OPENSIM_WITH_CASADI variable during the install step.
    install(CODE "set(OPENSIM_WITH_CASADI \"${OPENSIM_WITH_CASADI}\")")
    install(SCRIPT "${script}")

endif()
