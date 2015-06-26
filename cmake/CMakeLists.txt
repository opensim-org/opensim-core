# CMake packaging so people can use OpenSim.
# ------------------------------------------

# Requires CMake 2.8.8.
include(CMakePackageConfigHelpers)

set(OPENSIM_INSTALL_CMAKE_DIR cmake)

configure_file(OpenSimConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfig.cmake" @ONLY)

configure_package_config_file(
    OpenSimConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfig.cmake
    INSTALL_DESTINATION "${OPENSIM_INSTALL_CMAKE_DIR}"
    PATH_VARS # Variables to edit in OpenSimConfig.cmake.in.
        CMAKE_INSTALL_PREFIX
        OPENSIM_INSTALL_CMAKE_DIR
    )

# Version file.
write_basic_config_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfigVersion.cmake"
    VERSION "${OPENSIM_VERSION}"
    COMPATIBILITY SameMajorVersion)

install(
    FILES
        "SampleCMakeLists.txt"
        "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfigVersion.cmake"
    DESTINATION
        "${OPENSIM_INSTALL_CMAKE_DIR}"
        )

install(EXPORT OpenSimTargets DESTINATION "${OPENSIM_INSTALL_CMAKE_DIR}")
