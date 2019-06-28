MocoAddExampleCXX(NAME exampleMocoTrack
                  RESOURCES
                    coordinates.sto
                    grf_walk.mot
                    grf_walk.xml
                    marker_trajectories.trc
                    subject_walk_armless.osim)

set(SANDBOX_MOCO_TRACK_DIR 
    "${CMAKE_BINARY_DIR}/Moco/Sandbox/sandboxMocoTrack")
file(COPY "coordinates.sto"
          "grf_walk.mot"
          "grf_walk.xml"
          "marker_trajectories.trc"
          "subject_walk_armless.osim"
     DESTINATION "${SANDBOX_MOCO_TRACK_DIR}")
     
set(EXAMPLE_MOCO_TRACK_MATLAB_BINARY_DIR 
    "${CMAKE_BINARY_DIR}/Moco/Bindings/Java/Matlab/Examples/exampleMocoTrack")
file(COPY "coordinates.sto"
          "grf_walk.mot"
          "grf_walk.xml"
          "marker_trajectories.trc"
          "subject_walk_armless.osim"
     DESTINATION "${EXAMPLE_MOCO_TRACK_MATLAB_BINARY_DIR}")
     
set(EXAMPLE_MOCO_TRACK_MATLAB_INSTALL_DIR 
    "${MOCO_INSTALL_APIEXDIR}/Matlab/exampleMocoTrack")
install(FILES "coordinates.sto"
              "grf_walk.mot"
              "grf_walk.xml"
              "marker_trajectories.trc"
              "subject_walk_armless.osim"
        DESTINATION "${EXAMPLE_MOCO_TRACK_MATLAB_INSTALL_DIR}")
    
set(EXAMPLE_MOCO_TRACK_PYTHON_BINARY_DIR 
    "${CMAKE_BINARY_DIR}/Moco/Bindings/Python/examples/exampleMocoTrack")
file(COPY "coordinates.sto"
          "grf_walk.mot"
          "grf_walk.xml"
          "marker_trajectories.trc"
          "subject_walk_armless.osim"
     DESTINATION "${EXAMPLE_MOCO_TRACK_PYTHON_BINARY_DIR}")
     
set(EXAMPLE_MOCO_TRACK_PYTHON_INSTALL_DIR 
    "${MOCO_INSTALL_APIEXDIR}/Python/exampleMocoTrack")
install(FILES "coordinates.sto"
              "grf_walk.mot"
              "grf_walk.xml"
              "marker_trajectories.trc"
              "subject_walk_armless.osim"
        DESTINATION "${EXAMPLE_MOCO_TRACK_PYTHON_INSTALL_DIR}")