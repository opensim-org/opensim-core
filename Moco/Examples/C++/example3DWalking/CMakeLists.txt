set(EXAMPLE_MOCO_TRACK_FILES
        subject_walk_armless.osim
        coordinates.sto
        grf_walk.mot
        grf_walk.xml
        marker_trajectories.trc
        electromyography.sto)

MocoAddExampleCXX(NAME example3DWalking
        EXECUTABLES exampleMocoTrack exampleMocoInverse
        RESOURCES "${EXAMPLE_MOCO_TRACK_FILES}")

file(GLOB GEOMETRY "${CMAKE_SOURCE_DIR}/opensim-models/Geometry/*"
    "${CMAKE_SOURCE_DIR}/opensim-models/Models/RajagopalModel/Geometry/*")

file(COPY ${GEOMETRY} 
        DESTINATION
        "${CMAKE_BINARY_DIR}/Moco/Examples/C++/example3DWalking/Geometry")
install(FILES ${GEOMETRY}
        DESTINATION "${MOCO_INSTALL_CPPEXDIR}/example3DWalking/Geometry")

file(COPY ${EXAMPLE_MOCO_TRACK_FILES}
        DESTINATION "${CMAKE_BINARY_DIR}/Moco/Sandbox/sandboxMocoTrack")

file(COPY ${EXAMPLE_MOCO_TRACK_FILES}
        DESTINATION
        "${CMAKE_BINARY_DIR}/Moco/Bindings/Java/Matlab/Examples/example3DWalking")
install(FILES ${EXAMPLE_MOCO_TRACK_FILES}
        DESTINATION "${MOCO_INSTALL_MATLABEXDIR}/example3DWalking")
install(FILES ${GEOMETRY}
        DESTINATION "${MOCO_INSTALL_MATLABEXDIR}/example3DWalking/Geometry")

file(COPY ${EXAMPLE_MOCO_TRACK_FILES}
        DESTINATION
        "${CMAKE_BINARY_DIR}/Moco/Bindings/Python/examples/example3DWalking")
install(FILES ${EXAMPLE_MOCO_TRACK_FILES}
        DESTINATION "${MOCO_INSTALL_PYTHONEXDIR}/example3DWalking")
install(FILES ${GEOMETRY}
        DESTINATION "${MOCO_INSTALL_PYTHONEXDIR}/example3DWalking/Geometry")
