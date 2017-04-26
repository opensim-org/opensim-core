
file(GLOB SOURCES *.cpp)
file(GLOB INCLUDES *.h)

OpenSimAddLibrary(
    KIT SimmFileWriter
    AUTHORS "Peter_Loan"
    EXCLUDEFROMPYTHON
    LINKLIBS osimCommon osimSimulation osimActuators ${Simbody_LIBRARIES}
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS ""
    )
