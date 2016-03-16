
if(Matlab_FOUND)
    add_subdirectory(tests)
endif()

# Install.
# --------
install(DIRECTORY examples/ DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}")

# This script contains paths into the OpenSim installation that may be
# different on different platforms, so we configure it with CMake variables.
configure_file(configureOpenSim.m.in
    "${CMAKE_CURRENT_BINARY_DIR}/configureOpenSim.m" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/configureOpenSim.m"
        DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}")
