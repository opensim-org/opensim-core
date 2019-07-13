set(EXAMPLE_MOCO_COORDINATE_TRACKING_FILES
      gait10dof18musc_MusclePolynomials.osim
      gait10dof18musc_MuscleGeometryPath.osim
      gait10dof18musc_CoordinateActuators.osim
      referenceCoordinate.sto)

MocoAddExampleCXX(NAME exampleCoordinateTracking
                  RESOURCES "${EXAMPLE_MOCO_COORDINATE_TRACKING_FILES}")
