
file(GLOB INCLUDES *.h)
file(GLOB SOURCES *.cpp)

OpenSimAddLibrary(
    KIT Analyses
    AUTHORS "Clay_Anderson-Ayman_Habib-Peter_Loan"
    LINKLIBS osimCommon osimSimulation osimActuators
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS Test
    )
