set(EXAMPLE_MOCO_TRACK_FILES
        coordinates.sto
        grf_walk.mot
        grf_walk.xml
        marker_trajectories.trc
        subject_walk_armless.osim)

MocoAddExampleCXX(NAME exampleMocoTrack
        RESOURCES "${EXAMPLE_MOCO_TRACK_FILES}")

file(GLOB GEOMETRY "${CMAKE_SOURCE_DIR}/opensim-models/Geometry/*"
    "${CMAKE_SOURCE_DIR}/opensim-models/Models/RajagopalModel/Geometry/*")

file(COPY ${GEOMETRY} 
        DESTINATION
        "${CMAKE_BINARY_DIR}/Moco/Examples/C++/exampleMocoTrack/Geometry")
install(FILES ${GEOMETRY}
        DESTINATION "${MOCO_INSTALL_CPPEXDIR}/exampleMocoTrack/Geometry")

file(COPY ${EXAMPLE_MOCO_TRACK_FILES}
        DESTINATION "${CMAKE_BINARY_DIR}/Moco/Sandbox/sandboxMocoTrack")

file(COPY ${EXAMPLE_MOCO_TRACK_FILES}
        DESTINATION
        "${CMAKE_BINARY_DIR}/Moco/Bindings/Java/Matlab/Examples/exampleMocoTrack")
install(FILES ${EXAMPLE_MOCO_TRACK_FILES}
        DESTINATION "${MOCO_INSTALL_MATLABEXDIR}/exampleMocoTrack")
install(FILES ${GEOMETRY}
        DESTINATION "${MOCO_INSTALL_MATLABEXDIR}/exampleMocoTrack/Geometry")

file(COPY ${EXAMPLE_MOCO_TRACK_FILES}
        DESTINATION
        "${CMAKE_BINARY_DIR}/Moco/Bindings/Python/examples/exampleMocoTrack")
install(FILES ${EXAMPLE_MOCO_TRACK_FILES}
        DESTINATION "${MOCO_INSTALL_PYTHONEXDIR}/exampleMocoTrack")
install(FILES ${GEOMETRY}
        DESTINATION "${MOCO_INSTALL_PYTHONEXDIR}/exampleMocoTrack/Geometry")
