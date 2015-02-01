
FILE(GLOB SOURCES *.cpp)
FILE(GLOB INCLUDES *.h)

OPENSIM_ADD_LIBRARY(
    KIT SimmFileWriter
    AUTHORS "Peter_Loan"
    LINKLIBS osimCommon osimSimulation osimActuators ${SIMTK_ALL_LIBS}
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS ""
    )
