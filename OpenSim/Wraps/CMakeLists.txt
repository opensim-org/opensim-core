
file(GLOB INCLUDES *.h)
file(GLOB SOURCES *.cpp)

OpenSimAddLibrary(
    KIT Wraps
    AUTHORS "Ajay_Seth-Pariterre-Ayman_Habib"
    LINKLIBS osimCommon
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    )
