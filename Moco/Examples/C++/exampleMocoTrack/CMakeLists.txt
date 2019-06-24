
MocoAddExampleCXX(NAME exampleMocoTrack
                  RESOURCES
                    coordinates_rra_adjusted.sto
                    grf_walk.mot
                    grf_walk.xml
                    motion_capture_walk.trc
                    subject_walk_rra_adjusted_armless.osim)

set(SANDBOX_MOCO_TRACK_DIR 
    "${CMAKE_BINARY_DIR}/Moco/Sandbox/sandboxMocoTrack")
file(COPY "coordinates_rra_adjusted.sto"
          "grf_walk.mot"
          "grf_walk.xml"
          "motion_capture_walk.trc"
          "subject_walk_rra_adjusted_armless.osim"
     DESTINATION "${SANDBOX_MOCO_TRACK_DIR}")
     
set(EXAMPLE_MOCO_TRACK_MATLAB_DIR 
    "${CMAKE_BINARY_DIR}/Moco/Bindings/Java/Matlab/Examples/exampleMocoTrack")
file(COPY "coordinates_rra_adjusted.sto"
          "grf_walk.mot"
          "grf_walk.xml"
          "motion_capture_walk.trc"
          "subject_walk_rra_adjusted_armless.osim"
     DESTINATION "${EXAMPLE_MOCO_TRACK_MATLAB_DIR}")
    
set(EXAMPLE_MOCO_TRACK_PYTHON_DIR 
    "${CMAKE_BINARY_DIR}/Moco/Bindings/Python/examples/exampleMocoTrack")
file(COPY "coordinates_rra_adjusted.sto"
          "grf_walk.mot"
          "grf_walk.xml"
          "motion_capture_walk.trc"
          "subject_walk_rra_adjusted_armless.osim"
     DESTINATION "${EXAMPLE_MOCO_TRACK_PYTHON_DIR}")