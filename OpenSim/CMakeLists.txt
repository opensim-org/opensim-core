#-----------------------------------------------------------------------------
SET(CMAKE_DEBUG_POSTFIX "_d" CACHE INTERNAL "" FORCE)

IF (BUILD_UTILITIES)
SUBDIRS(Common Simulation Actuators Analyses Models Utilities Tools Java Applications Examples Tests)
ELSE (BUILD_UTILITIES)
SUBDIRS(Common Simulation Actuators Analyses Models Tools Java Applications Examples Tests)
ENDIF (BUILD_UTILITIES)

INSTALL_FILES(/sdk/include/OpenSim .h OpenSim.h)
# INCLUDE(ApiDoxygen.cmake)