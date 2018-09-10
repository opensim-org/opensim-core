
file(GLOB INCLUDES *.h)
file(GLOB SOURCES *.cpp)

OpenSimAddLibrary(
    KIT Wraps
    AUTHORS "Pariterre"
    LINKLIBS osimCommon
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    #TESTDIRS "Test"
    )
