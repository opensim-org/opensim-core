
find_package(Matlab COMPONENTS MAIN_PROGRAM)
if(Matlab_FOUND AND BUILD_TESTING)
    add_subdirectory(Tests)
endif()

install(DIRECTORY Examples/ DESTINATION "${MUSCOLLO_INSTALL_MATLABEXDIR}")
set(SANDBOX_MARKER_TRACKING_DIR 
    "${CMAKE_SOURCE_DIR}/Muscollo/Sandbox/sandboxMarkerTrackingWholeBody")

install(FILES 
            "${SANDBOX_MARKER_TRACKING_DIR}/marker_trajectories.trc"
            "${SANDBOX_MARKER_TRACKING_DIR}/subject01.osim"
        DESTINATION 
            "${MUSCOLLO_INSTALL_MATLABEXDIR}/exampleMarkerTracking_gait1018")

# The configureMuscollo.m script contains paths into the Muscollo installation
# that may be different on different platforms, so we configure it with CMake
# variables.
file(RELATIVE_PATH matlabexdir_to_install_dir
    "${CMAKE_INSTALL_PREFIX}/${MUSCOLLO_INSTALL_MATLABEXDIR}"
    "${CMAKE_INSTALL_PREFIX}")
configure_file(configureMuscollo.m.in
    "${CMAKE_CURRENT_BINARY_DIR}/configureMuscollo.m" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/configureMuscollo.m"
        DESTINATION "${MUSCOLLO_INSTALL_MATLABEXDIR}")
