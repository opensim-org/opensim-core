
file(GLOB INCLUDES *.h)
file(GLOB SOURCES *.cpp)

OpenSimAddLibrary(
    KIT ExampleComponents
    AUTHORS "Ajay_Seth-Michael_Sherman-Ayman_Habib"
    LINKLIBS ${Simbody_LIBRARIES} osimActuators osimWraps
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    )
