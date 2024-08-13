add_subdirectory(lepton)
if(OPENSIM_WITH_TROPTER OR OPENSIM_COPY_DEPENDENCIES)
    add_subdirectory(tropter)
endif()
