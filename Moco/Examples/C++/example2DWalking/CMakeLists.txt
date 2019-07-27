set(EXAMPLE_MOCO_2D_WALKING_FILES
      2D_gait.osim
      referenceCoordinates.sto)

MocoAddExampleCXX(NAME example2DWalking
                  RESOURCES "${EXAMPLE_MOCO_2D_WALKING_FILES}")
