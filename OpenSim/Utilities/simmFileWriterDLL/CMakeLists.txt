
FILE(GLOB SOURCES *.cpp)
FILE(GLOB INCLUDES *.h)

OpenSimAddLibrary(
    KIT SimmFileWriter
    AUTHORS "Peter_Loan"
    LINKLIBS osimCommon osimSimulation osimActuators ${Simbody_LIBRARIES}
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS ""
    )
