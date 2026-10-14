OpenSimAddExampleCXX(NAME examplePolynomialPathFitter SUBDIR PolynomialPathFitter)

# Copy resources from example3DWalking to the source directory.
set(EXAMPLE_3D_WALKING_DIR
    "${CMAKE_SOURCE_DIR}/OpenSim/Examples/Moco/example3DWalking")
set(EXAMPLE_PATH_FITTER_FILES
    "${EXAMPLE_3D_WALKING_DIR}/subject_walk_scaled.osim"
    "${EXAMPLE_3D_WALKING_DIR}/coordinates.sto")

# Copy and install example resources.
file(COPY ${EXAMPLE_PATH_FITTER_FILES}
     DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
install(FILES ${EXAMPLE_PATH_FITTER_FILES}
        DESTINATION "${OPENSIM_INSTALL_CPPEXDIR}/PolynomialPathFitter/examplePolynomialPathFitter")
