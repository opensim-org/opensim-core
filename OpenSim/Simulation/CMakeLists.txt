
set(SIMULATION_SUBDIRS 
        .
        Control
        Manager
        Model
        Wrap
        SimbodyEngine)

set(SOURCES)
set(INCLUDES)
foreach(dir ${SIMULATION_SUBDIRS})
    file(GLOB SOURCES_ONEDIR ${dir}/*.cpp)
    file(GLOB INCLUDES_ONEDIR ${dir}/*.h)
    list(APPEND SOURCES ${SOURCES_ONEDIR})
    list(APPEND INCLUDES ${INCLUDES_ONEDIR})
endforeach()

# Generate precompiled header. This macro creates a target named 
# 'SimulationPCH'.
GeneratePCH(osimSimulation.h Simulation)

OpenSimAddLibrary(KIT Simulation
    LINKLIBS ${Simbody_LIBRARIES} osimCommon osimLepton
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS SimbodyEngine Test
    INCLUDEDIRS ${SIMULATION_SUBDIRS})
add_dependencies(osimSimulation SimulationPCH)