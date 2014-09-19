# CMake packaging so people can use OpenSim.
# ------------------------------------------

SET(OPENSIM_INSTALL_CMAKE_DIR cmake)

CONFIGURE_FILE(OpenSimConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfig.cmake" @ONLY)

# Version file.
include(WriteBasicConfigVersionFile)
WRITE_BASIC_CONFIG_VERSION_FILE(
    "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfigVersion.cmake"
    VERSION "${OPENSIM_VERSION}"
    COMPATIBILITY SameMajorVersion)

INSTALL(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/OpenSimConfigVersion.cmake"
    DESTINATION
        "${OPENSIM_INSTALL_CMAKE_DIR}"
        )

install(EXPORT OpenSimTargets DESTINATION "${OPENSIM_INSTALL_CMAKE_DIR}")
