
file(GLOB INCLUDES *.h gcvspl.h)
file(GLOB SOURCES *.cpp gcvspl.c)

if(NOT WITH_EZC3D)
    file(GLOB C3D_HEADER *C3DFileAdapter.h)
    file(GLOB C3D_SOURCE *C3DFileAdapter.cpp)
    list(REMOVE_ITEM INCLUDES ${C3D_HEADER})
    list(REMOVE_ITEM SOURCES  ${C3D_SOURCE})
endif()
if (NOT WITH_EZC3D)
    unset(ezc3d_LIBRARY)
endif()

find_package(FastFloat REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/fast_float")
OpenSimAddLibrary(
    KIT Common
    AUTHORS "Clay_Anderson-Ayman_Habib-Peter_Loan"
    # Clients of osimCommon need not link to ezc3d.
    LINKLIBS PUBLIC ${Simbody_LIBRARIES} spdlog::spdlog osimLepton 
             PRIVATE ${ezc3d_LIBRARY} FastFloat::fast_float  
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    TESTDIRS "Test"
    )

if(WIN32)
    # On Windows only, debug libraries cannot be mixed with release
    # libraries, and we must copy the DLLs from the dependencies
    # into OpenSim's build directory (so that the DLLs are found
    # when running the tests).
    add_dependencies(osimCommon
        Simbody_CONFIG_check Copy_Simbody_DLLs)
    if(WITH_EZC3D)
        add_dependencies(osimCommon Copy_ezc3d_DLLs)
    endif()
endif()
  
