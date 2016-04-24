
file(GLOB INCLUDES *.h gcvspl.h)
file(GLOB SOURCES *.cpp gcvspl.c)
file(GLOB C3D_HEADER *C3DFileAdapter.h)
file(GLOB C3D_SOURCE *C3DFileAdapter.cpp)
    
if(NOT WITH_BTK)
    list(REMOVE_ITEM INCLUDES ${C3D_HEADER})
    list(REMOVE_ITEM SOURCES  ${C3D_SOURCE})
    unset(BTK_LIBRARIES)
endif()

OpenSimAddLibrary(
    UNITY_EXCLUDE
    KIT Common
    AUTHORS "Clay_Anderson-Ayman_Habib-Peter_Loan"
    LINKLIBS ${Simbody_LIBRARIES} ${BTK_LIBRARIES}
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS "Test"
    UNITY_EXCLUDED_SOURCES ${C3D_SOURCE}
    )

    
