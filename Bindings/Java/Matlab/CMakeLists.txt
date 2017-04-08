
if(Matlab_FOUND)
    if(BUILD_TESTING)
        add_subdirectory(tests)
    endif()
endif()

# Install.
# --------
install(DIRECTORY examples/ DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}")
install(DIRECTORY Hopper_Device DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}")
install(DIRECTORY Utilities DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}")

# The configureOpenSim.m script contains paths into the OpenSim installation
# that may be different on different platforms, so we configure it with CMake
# variables.
file(RELATIVE_PATH matlabexdir_to_install_dir
    "${CMAKE_INSTALL_PREFIX}/${OPENSIM_INSTALL_MATLABEXDIR}"
    "${CMAKE_INSTALL_PREFIX}")
configure_file(configureOpenSim.m.in
    "${CMAKE_CURRENT_BINARY_DIR}/configureOpenSim.m" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/configureOpenSim.m"
        DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}")
