
FILE(GLOB INCLUDES *.h gcvspl.h)
FILE(GLOB SOURCES *.cpp gcvspl.c)

OPENSIM_ADD_LIBRARY(
    KIT Common
    AUTHORS "Clay_Anderson-Ayman_Habib-Peter_Loan"
    LINKLIBS ${Simbody_LIBRARIES}
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS "Test"
    )

    
