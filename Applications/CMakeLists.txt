

if(OPENSIM_BUILD_INDIVIDUAL_APPS)
    # Basically, don't install the old applications on Linux or OSX, where
    # they conflict with system executables like "id".
    add_subdirectory(Analyze)
    add_subdirectory(Forward)
    add_subdirectory(Scale)
    add_subdirectory(IK)
    add_subdirectory(ID)
    add_subdirectory(CMC)
    add_subdirectory(RRA)
    add_subdirectory(versionUpdate)
endif()

add_subdirectory(opensim)
