set(EXAMPLE_2D_WALKING_FILES
        2D_gait.osim
        referenceCoordinates.sto
        referenceGRF.sto
        referenceGRF.xml)

OpenSimAddExampleCXX(NAME example2DWalking SUBDIR Moco
    RESOURCES "${EXAMPLE_2D_WALKING_FILES}")

file(COPY ${EXAMPLE_2D_WALKING_FILES}
        DESTINATION
        "${CMAKE_BINARY_DIR}/Bindings/Java/Matlab/examples/Moco/example2DWalking")
install(FILES ${EXAMPLE_2D_WALKING_FILES}
    DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}/Moco/example2DWalking")
