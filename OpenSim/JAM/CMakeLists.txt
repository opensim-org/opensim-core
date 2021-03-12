#CMakeLists.txt for JAM
#======================

set(JAM_SOURCES
        About.cpp
        base64.cpp
        COMAKInverseKinematicsTool.cpp
        COMAKTarget.cpp
        COMAKTool.cpp
        ForsimTool.cpp
        H5FileAdapter.cpp
        JointMechanicsTool.cpp
        JAMUtilities.cpp
        RegisterTypes_osimJAM.cpp
        VTPFileAdapter.cpp
        )
        
set(JAM_INCLUDES
        About.h
        base64.h
        COMAKInverseKinematicsTool.h
        COMAKTarget.h
        COMAKTool.h
        ForsimTool.h
        H5FileAdapter.h
        JointMechanicsTool.h
        JAMUtilities.h
        osimJAM.h
        osimJAMDLL.h
        RegisterTypes_osimJAM.h
        VTPFileAdapter.h
        )

OpenSimAddLibrary(
    KIT JAM
    AUTHORS "Colin_Smith"
    # Clients of osimCommon need not link to BTK or ezc3d.
    LINKLIBS PUBLIC ${Simbody_LIBRARIES} spdlog::spdlog osimTools osimCommon
             PRIVATE hdf5_cpp-shared hdf5_hl-shared
    INCLUDES ${JAM_INCLUDES}
    SOURCES ${JAM_SOURCES}
    )
