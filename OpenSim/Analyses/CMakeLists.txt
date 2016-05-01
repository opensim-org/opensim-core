
file(GLOB INCLUDES *.h)
file(GLOB SOURCES *.cpp)

# Generate precompiled header. This macro creates a target named 'CommonPCH'.
GeneratePCH(osimAnalyses.h Analyses)

OpenSimAddLibrary(
    KIT Analyses
    AUTHORS "Clay_Anderson-Ayman_Habib-Peter_Loan"
    LINKLIBS osimCommon osimSimulation osimActuators
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    )
add_dependencies(osimAnalyses AnalysesPCH)
