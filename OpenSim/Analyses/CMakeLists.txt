
FILE(GLOB INCLUDES *.h)
FILE(GLOB SOURCES *.cpp)

OPENSIM_ADD_LIBRARY(
    KIT Analyses
    AUTHORS "Clay_Anderson-Ayman_Habib-Peter_Loan"
    LINKLIBS osimCommon osimSimulation osimActuators
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    )
