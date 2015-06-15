
FILE(GLOB INCLUDES *.h gcvspl.h)
FILE(GLOB SOURCES *.cpp gcvspl.c)

OpenSimAddLibrary(
    KIT Common
    AUTHORS "Clay_Anderson-Ayman_Habib-Peter_Loan"
    LINKLIBS ${Simbody_LIBRARIES}
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS "Test"
    )

    
