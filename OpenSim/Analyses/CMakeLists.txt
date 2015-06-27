
file(GLOB INCLUDES *.h)
file(GLOB SOURCES *.cpp)

OPENSIM_add_library(
    KIT Analyses
    AUTHORS "Clay_Anderson-Ayman_Habib-Peter_Loan"
    LINKLIBS osimCommon osimSimulation osimActuators
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    )
