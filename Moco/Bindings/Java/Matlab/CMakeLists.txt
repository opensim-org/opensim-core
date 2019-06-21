
find_package(Matlab COMPONENTS MAIN_PROGRAM)
if(Matlab_FOUND AND BUILD_TESTING)
    add_subdirectory(Tests)
endif()

install(DIRECTORY Examples/ DESTINATION "${MOCO_INSTALL_MATLABEXDIR}")

# Copy example models and data files to use in Sandbox examples.
set(SANDBOX_MARKER_TRACKING_DIR 
    "${CMAKE_BINARY_DIR}/Moco/Sandbox/sandboxMarkerTrackingWholeBody")
file(COPY "Examples/exampleMarkerTracking10DOF/marker_trajectories.trc"
          "Examples/exampleMarkerTracking10DOF/subject01.osim"
          "Examples/exampleMarkerTracking10DOF/state_reference.mot"
     DESTINATION "${SANDBOX_MARKER_TRACKING_DIR}")
 
set(SANDBOX_MOCO_TRACK_DIR 
    "${CMAKE_BINARY_DIR}/Moco/Sandbox/sandboxMocoTrack")
file(COPY "Examples/exampleMocoTrack/coordinates_rra_adjusted.sto"
          "Examples/exampleMocoTrack/grf_walk.mot"
          "Examples/exampleMocoTrack/grf_walk.xml"
          "Examples/exampleMocoTrack/motion_capture_walk.trc"
          "Examples/exampleMocoTrack/subject_walk_rra_adjusted_armless.osim"
     DESTINATION "${SANDBOX_MOCO_TRACK_DIR}")
     
set(EXAMPLE_MOCO_TRACK_DIR 
    "${CMAKE_BINARY_DIR}/Moco/Examples/C++/exampleMocoTrack")
file(COPY "Examples/exampleMocoTrack/coordinates_rra_adjusted.sto"
          "Examples/exampleMocoTrack/grf_walk.mot"
          "Examples/exampleMocoTrack/grf_walk.xml"
          "Examples/exampleMocoTrack/motion_capture_walk.trc"
          "Examples/exampleMocoTrack/subject_walk_rra_adjusted_armless.osim"
     DESTINATION "${EXAMPLE_MOCO_TRACK_DIR}")
    
set(EXAMPLE_MOCO_TRACK_PYTHON_DIR 
    "${CMAKE_BINARY_DIR}/Moco/Bindings/Python/examples/exampleMocoTrack")
file(COPY "Examples/exampleMocoTrack/coordinates_rra_adjusted.sto"
          "Examples/exampleMocoTrack/grf_walk.mot"
          "Examples/exampleMocoTrack/grf_walk.xml"
          "Examples/exampleMocoTrack/motion_capture_walk.trc"
          "Examples/exampleMocoTrack/subject_walk_rra_adjusted_armless.osim"
     DESTINATION "${EXAMPLE_MOCO_TRACK_PYTHON_DIR}")
    
# The configureMoco.m script contains paths into the Moco installation
# that may be different on different platforms, so we configure it with CMake
# variables.
file(RELATIVE_PATH matlabexdir_to_install_dir
    "${CMAKE_INSTALL_PREFIX}/${MOCO_INSTALL_MATLABEXDIR}"
    "${CMAKE_INSTALL_PREFIX}")
configure_file(configureMoco.m.in
    "${CMAKE_CURRENT_BINARY_DIR}/configureMoco.m" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/configureMoco.m"
        DESTINATION "${MOCO_INSTALL_MATLABEXDIR}")
