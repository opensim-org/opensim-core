
FILE(GLOB INCLUDES *.h)
FILE(GLOB SOURCES *.cpp)

OPENSIM_ADD_LIBRARY(
    KIT Actuators
    LINKLIBS osimCommon osimSimulation
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS "Test"
    )
