
FILE(GLOB INCLUDES *.h)
FILE(GLOB SOURCES *.cpp)

OpenSimAddLibrary(
    KIT Actuators
    AUTHORS "Ajay_Seth-Matt_Millard-Matt_Demers-Ayman_Habib"
    LINKLIBS osimCommon osimSimulation
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS "Test"
    )
