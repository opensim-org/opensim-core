find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

OpenSimAddTest(NAME testOpenSense
        LINKLIBS osimTools
        RESOURCES resources/MT_012005D6_009-quaternions_RHJCSwinger.sto
                  resources/MT_012005D6_009-quaternions_calibration_trial_Facing_X.sto
                  resources/MT_012005D6_009-quaternions_calibration_trial_Facing_negX.sto
                  resources/calibrate_rotated.xml
                  resources/imuOrientations.sto
                  resources/imuPlacer.xml
                  resources/imuPlacerFaceNegX.xml
                  resources/imuPlacerFaceX.xml
                  resources/model_Rajagopal2015_posed.osim
                  resources/myComplexTasksTrial_orientations.sto
                  resources/setup_IMUInverseKinematics_HJC_trial.xml
                  resources/setup_IMUInverseKinematics_HJC_trial_nofeet.xml
                  resources/std_calibrated_pelvis_rot.osim
                  resources/std_calibrated_subject07.osim
                  resources/subject07.osim
        )
