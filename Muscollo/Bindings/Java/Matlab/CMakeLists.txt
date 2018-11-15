
find_package(Matlab COMPONENTS MAIN_PROGRAM)
if(Matlab_FOUND AND BUILD_TESTING)
    add_subdirectory(Tests)
endif()

install(DIRECTORY Examples/ DESTINATION "${MUSCOLLO_INSTALL_MATLABEXDIR}")

# Copy example model and data file to use in Sandbox example.
set(SANDBOX_MARKER_TRACKING_DIR 
    "${CMAKE_BINARY_DIR}/Muscollo/Sandbox/sandboxMarkerTrackingWholeBody")
file(COPY "Examples/exampleMarkerTracking10DOF/marker_trajectories.trc"
		  "Examples/exampleMarkerTracking10DOF/subject01.osim"
	 DESTINATION "${SANDBOX_MARKER_TRACKING_DIR}")
	
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
