
FILE(GLOB INCLUDES *.h)
FILE(GLOB SOURCES *.cpp)

OPENSIM_ADD_LIBRARY(
    KIT Actuators
    AUTHORS "Ajay_Seth-Matt_Millard-Matt-Demers-Ayman_Habib"
    LINKLIBS osimCommon osimSimulation
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS "Test"
    )
