
file(GLOB SOURCES *.cpp)
file(GLOB INCLUDES *.h)

OPENSIM_add_library(
    KIT SimmFileWriter
    AUTHORS "Peter_Loan"
    LINKLIBS osimCommon osimSimulation osimActuators ${Simbody_LIBRARIES}
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS ""
    )
