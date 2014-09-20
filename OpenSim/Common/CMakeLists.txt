
FILE(GLOB INCLUDES *.h gcvspl.h)
FILE(GLOB SOURCES *.cpp gcvspl.c)

OPENSIM_ADD_LIBRARY(
    KIT Common
    AUTHORS "Clay_Anderson-Ayman_Habib_and_Peter_Loan"
    LINKLIBS ${SIMTK_COMMON_LIB} ${SIMTK_MATH_LIB} ${MATH_LIBS_TO_USE}
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS "Test"
    )

    
