set(EXAMPLE_MOCO_TRACK_FILES
        subject_walk_scaled.osim
        subject_walk_scaled_FunctionBasedPathSet.xml
        coordinates.sto
        grf_walk.mot
        grf_walk.xml
        marker_trajectories.trc
        electromyography.sto
        README.txt)

OpenSimAddExampleCXX(NAME example3DWalking SUBDIR Moco
        EXECUTABLES exampleMocoTrack exampleMocoInverse
        RESOURCES "${EXAMPLE_MOCO_TRACK_FILES}")

file(COPY ${EXAMPLE_MOCO_TRACK_FILES}
        DESTINATION "${CMAKE_BINARY_DIR}/Moco/Sandbox/sandboxMocoTrack")

file(COPY ${EXAMPLE_MOCO_TRACK_FILES}
        DESTINATION
        "${CMAKE_BINARY_DIR}/Bindings/Java/Matlab/examples/Moco/example3DWalking")
install(FILES ${EXAMPLE_MOCO_TRACK_FILES}
    DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}/Moco/example3DWalking")

file(COPY ${EXAMPLE_MOCO_TRACK_FILES}
        DESTINATION
        "${CMAKE_BINARY_DIR}/Bindings/Python/examples/Moco/example3DWalking")
install(FILES ${EXAMPLE_MOCO_TRACK_FILES}
        DESTINATION "${OPENSIM_INSTALL_PYTHONEXDIR}/Moco/example3DWalking")

set(EXAMPLE_PATH_FITTER_FILES
        subject_walk_scaled.osim
        coordinates.sto)

file(COPY ${EXAMPLE_PATH_FITTER_FILES}
        DESTINATION
        "${CMAKE_BINARY_DIR}/Bindings/Python/examples/PolynomialPathFitter")
install(FILES ${EXAMPLE_PATH_FITTER_FILES}
        DESTINATION "${OPENSIM_INSTALL_PYTHONEXDIR}/PolynomialPathFitter")

file(COPY ${EXAMPLE_PATH_FITTER_FILES}
        DESTINATION
        "${CMAKE_BINARY_DIR}/Bindings/Java/Matlab/examples/PolynomialPathFitter")
install(FILES ${EXAMPLE_PATH_FITTER_FILES}
        DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}/PolynomialPathFitter")
