#-----------------------------------------------------------------------------
SET(CMAKE_DEBUG_POSTFIX "_d" CACHE INTERNAL "" FORCE)

SUBDIRS(Common Simulation SQP DynamicsEngines Actuators Analyses Models Utilities Tools Applications Examples Tests)
