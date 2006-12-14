#-----------------------------------------------------------------------------
SET(CMAKE_DEBUG_POSTFIX "_d" CACHE INTERNAL "" FORCE)

IF(OPENSIM_MINIMAL_BUILD)
SUBDIRS(Tools Simulation SQP CMC Actuators Analyses Models Applications Examples)
ELSE(OPENSIM_MINIMAL_BUILD)
SUBDIRS(Tools Simulation SQP CMC Actuators Analyses Models Utilities Applications Examples)
ENDIF(OPENSIM_MINIMAL_BUILD)
