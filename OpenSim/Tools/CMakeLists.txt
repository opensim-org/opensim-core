
FILE(GLOB INCLUDES *.h)
FILE(GLOB SOURCES *.cpp)

OPENSIM_ADD_LIBRARY(
    KIT Tools
    LINKLIBS osimCommon osimSimulation osimActuators osimAnalyses
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS "Test"
    )
