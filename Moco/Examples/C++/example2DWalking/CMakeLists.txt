set(EXAMPLE_MOCO_2D_WALKING_FILES
      gait10dof18musc.osim
      referenceCoordinates.sto)

MocoAddExampleCXX(NAME example2DWalking
                  RESOURCES "${EXAMPLE_MOCO_2D_WALKING_FILES}")
