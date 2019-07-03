set(EXAMPLE_MOCO_TRACK_FILES
      coordinates.sto
      grf_walk.mot
      grf_walk.xml
      marker_trajectories.trc
      subject_walk_armless.osim)

MocoAddExampleCXX(NAME exampleMocoTrack
                  RESOURCES "${EXAMPLE_MOCO_TRACK_FILES}")
      
file(COPY ${EXAMPLE_MOCO_TRACK_FILES}
     DESTINATION "${CMAKE_BINARY_DIR}/Moco/Sandbox/sandboxMocoTrack")
     
file(COPY ${EXAMPLE_MOCO_TRACK_FILES}
     DESTINATION 
        "${CMAKE_BINARY_DIR}/Moco/Bindings/Java/Matlab/Examples/exampleMocoTrack") 
install(FILES ${EXAMPLE_MOCO_TRACK_FILES}
        DESTINATION "${MOCO_INSTALL_APIEXDIR}/Matlab/exampleMocoTrack")
    
file(COPY ${EXAMPLE_MOCO_TRACK_FILES}
     DESTINATION 
        "${CMAKE_BINARY_DIR}/Moco/Bindings/Python/examples/exampleMocoTrack")
install(FILES ${EXAMPLE_MOCO_TRACK_FILES}
        DESTINATION "${MOCO_INSTALL_APIEXDIR}/Python/exampleMocoTrack")