file(GLOB SOURCE_FILES src/*.cpp)

if(WIN32)
    add_definitions("-DLEPTON_BUILDING_SHARED_LIBRARY")
endif(WIN32)

include_directories(${OpenSim_SOURCE_DIR}/Vendors/lepton/include)

OpenSimAddLibrary(VENDORLIB LOWERINCLUDEDIRNAME
    KIT Lepton
    AUTHORS "Peter_Eastman"
    LINKLIBS ""
    INCLUDES "include/Lepton.h"
    SOURCES ${SOURCE_FILES}
    TESTDIRS test
    INCLUDEDIRS include include/lepton
    )
