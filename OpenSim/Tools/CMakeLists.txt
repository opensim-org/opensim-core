
FILE(GLOB INCLUDES *.h)
FILE(GLOB SOURCES *.cpp)

OPENSIM_ADD_LIBRARY(
    KIT Tools
    AUTHORS "Frank_C_Anderson"
    LINKLIBS osimCommon osimSimulation osimActuators osimAnalyses
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS "Test"
    )
