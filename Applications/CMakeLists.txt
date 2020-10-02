# If NOT OPENSIM_BUILD_INDIVIDUAL_APPS, then we will not build the old
# applications like "id". However, we still need to include those folders to
# build the tests for the tools.
add_subdirectory(Analyze)
add_subdirectory(Forward)
add_subdirectory(Scale)
add_subdirectory(IK)
add_subdirectory(ID)
add_subdirectory(CMC)
add_subdirectory(RRA)
add_subdirectory(opensense)
add_subdirectory(versionUpdate)

add_subdirectory(opensim-cmd)

if(UNIX)
    install(PROGRAMS opensim-install-command-line.sh DESTINATION
        ${CMAKE_INSTALL_BINDIR})
endif()
