
file(GLOB INCLUDES *.h gcvspl.h)
file(GLOB SOURCES *.cpp gcvspl.c)

OPENSIM_add_library(
    KIT Common
    AUTHORS "Clay_Anderson-Ayman_Habib-Peter_Loan"
    LINKLIBS ${Simbody_LIBRARIES}
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS "Test"
    )


