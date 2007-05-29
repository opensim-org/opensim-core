#-----------------------------------------------------------------------------
SET(CMAKE_DEBUG_POSTFIX "_d" CACHE INTERNAL "" FORCE)

SUBDIRS(Common Simulation DynamicsEngines Actuators Analyses Models Utilities Tools Java Applications Tests)
