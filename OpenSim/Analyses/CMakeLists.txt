
FILE(GLOB INCLUDES *.h)
FILE(GLOB SOURCES *.cpp)

OPENSIM_ADD_LIBRARY(
    KIT Analyses
    LINKLIBS osimCommon osimSimulation osimActuators
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    )
