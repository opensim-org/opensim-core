
file(GLOB INCLUDES *.h)
file(GLOB SOURCES *.cpp)

OpenSimAddLibrary(
    KIT ExampleComponents
    AUTHORS "Ajay_Seth-Michael_Sherman-Ayman_Habib"
    LINKLIBS ${Simbody_LIBRARIES} osimActuators
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    )

add_dependencies(osimExampleComponents Simbody_CONFIG_check)
