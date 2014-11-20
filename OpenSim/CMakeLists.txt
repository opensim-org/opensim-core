#-----------------------------------------------------------------------------

SET(OPENSIM_SYSTEM_INFO ${CMAKE_SYSTEM})
SET(OPENSIM_OS_NAME ${CMAKE_SYSTEM_NAME})

IF( WIN32 )
    SET(OPENSIM_COMPILER_INFO ${MSVC_VERSION})
ELSE ()
    SET(OPENSIM_COMPILER_INFO ${CMAKE_CXX_COMPILER} )
ENDIF()

ADD_DEFINITIONS(-DOSIM_SYS_INFO=${OPENSIM_SYSTEM_INFO}
-DOSIM_COMPILER_INFO=${OPENSIM_COMPILER_INFO}
-DOSIM_OS_NAME=${OPENSIM_OS_NAME})

SUBDIRS(Common Simulation Actuators Analyses Utilities Tools Wrapping Examples Tests doc)

INSTALL_FILES(/sdk/include/OpenSim .h OpenSim.h)