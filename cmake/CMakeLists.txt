# CMake packaging so people can use OpenSim.
# ------------------------------------------

# Requires CMake 2.8.8.
INCLUDE(CMakePackageConfigHelpers)

SET(OPENSIM_INSTALL_CMAKE_DIR cmake)

CONFIGURE_FILE(OpenSimConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfig.cmake" @ONLY)

CONFIGURE_PACKAGE_CONFIG_FILE(
    OpenSimConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfig.cmake
    INSTALL_DESTINATION "${OPENSIM_INSTALL_CMAKE_DIR}"
    PATH_VARS # Variables to edit in OpenSimConfig.cmake.in.
        CMAKE_INSTALL_PREFIX
        OPENSIM_INSTALL_CMAKE_DIR
    )

# Version file.
WRITE_BASIC_CONFIG_VERSION_FILE(
    "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfigVersion.cmake"
    VERSION "${OPENSIM_VERSION}"
    COMPATIBILITY SameMajorVersion)

INSTALL(
    FILES
        "SampleCMakeLists.txt"
        "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfigVersion.cmake"
    DESTINATION
        "${OPENSIM_INSTALL_CMAKE_DIR}"
        )

INSTALL(EXPORT OpenSimTargets DESTINATION "${OPENSIM_INSTALL_CMAKE_DIR}")
