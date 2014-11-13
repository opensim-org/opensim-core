
SET(SIMULATION_SUBDIRS 
        .
        Control
        Manager
        Model
        Wrap
        SimbodyEngine)

SET(SOURCES)
SET(INCLUDES)
FOREACH(dir ${SIMULATION_SUBDIRS})
    FILE(GLOB SOURCES_ONEDIR ${dir}/*.cpp)
    FILE(GLOB INCLUDES_ONEDIR ${dir}/*.h)
    LIST(APPEND SOURCES ${SOURCES_ONEDIR})
    LIST(APPEND INCLUDES ${INCLUDES_ONEDIR})
ENDFOREACH()

OPENSIM_ADD_LIBRARY(
    KIT Simulation
    AUTHORS "Ajay_Seth-Michael_Sherman-Ayman_Habib"
    LINKLIBS ${SIMTK_ALL_LIBS} osimCommon osimLepton
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS SimbodyEngine Test
    INCLUDEDIRS ${SIMULATION_SUBDIRS}
    )
