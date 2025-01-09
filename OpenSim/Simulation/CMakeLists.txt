
set(SIMULATION_SUBDIRS 
        .
        Control
        Manager
        Model
        Wrap
        SimbodyEngine
        OpenSense)

set(SOURCES)
set(INCLUDES)
foreach(dir ${SIMULATION_SUBDIRS})
    file(GLOB SOURCES_ONEDIR ${dir}/*.cpp)
    file(GLOB INCLUDES_ONEDIR ${dir}/*.h)
    list(APPEND SOURCES ${SOURCES_ONEDIR})
    list(APPEND INCLUDES ${INCLUDES_ONEDIR})
endforeach()

OpenSimAddLibrary(
    KIT Simulation
    AUTHORS "Ajay_Seth-Michael_Sherman-Ayman_Habib"
    LINKLIBS ${Simbody_LIBRARIES} osimCommon osimLepton
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS SimbodyEngine Test
    INCLUDEDIRS ${SIMULATION_SUBDIRS}
    )
