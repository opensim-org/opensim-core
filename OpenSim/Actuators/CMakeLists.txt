
file(GLOB INCLUDES *.h)
file(GLOB SOURCES *.cpp)

# Generate precompiled header. This macro creates a target named 'ActuatorsPCH'.
GeneratePCH(osimActuators.h Actuators)

OpenSimAddLibrary(
    KIT Actuators
    AUTHORS "Ajay_Seth-Matt_Millard-Matt_Demers-Ayman_Habib"
    LINKLIBS osimCommon osimSimulation
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS "Test"
    )
add_dependencies(osimActuators ActuatorsPCH)