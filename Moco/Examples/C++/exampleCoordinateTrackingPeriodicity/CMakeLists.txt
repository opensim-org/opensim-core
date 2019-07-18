set(EXAMPLE_MOCO_COORDINATE_TRACKING_PERIODICITY_FILES
      gait10dof18musc_CoordinateActuators.osim
      referenceCoordinate.sto)

MocoAddExampleCXX(NAME exampleCoordinateTrackingPeriodicity
                  RESOURCES "${EXAMPLE_MOCO_COORDINATE_TRACKING_PERIODICITY_FILES}")
