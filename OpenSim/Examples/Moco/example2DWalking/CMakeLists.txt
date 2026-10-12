set(EXAMPLE_2D_WALKING_FILES
        2D_gait.osim
        referenceCoordinates.sto
        referenceGRF.sto
        referenceGRF.xml)

OpenSimAddExampleCXX(NAME example2DWalking SUBDIR Moco
    EXECUTABLES example2DWalking example2DWalkingMetabolics
    RESOURCES "${EXAMPLE_2D_WALKING_FILES}")
