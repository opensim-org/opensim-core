find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

OpenSimAddTest(NAME testCMCArm26_Millard
        LINKLIBS osimTools
        RESOURCES resources/arm26.osim
                  resources/arm26_ComputedMuscleControl_Tasks.xml
                  resources/arm26_InverseKinematics.mot
                  resources/arm26_Reserve_Actuators.xml
                  resources/arm26_Setup_CMC.xml
                  resources/std_arm26_states.sto
        )

OpenSimAddTest(NAME testCMCArm26_Thelen
        LINKLIBS osimTools
        RESOURCES resources/arm26.osim
                  resources/arm26_ComputedMuscleControl_Tasks.xml
                  resources/arm26_InverseKinematics.mot
                  resources/arm26_Reserve_Actuators.xml
                  resources/arm26_Setup_CMC.xml
                  resources/std_arm26_states.sto
        )

OpenSimAddTest(NAME testCMCEMGDrivenArm_Millard
        LINKLIBS osimTools
        RESOURCES resources/arm26.osim
                  resources/arm26_ComputedMuscleControl_Tasks_EMG.xml
                  resources/arm26_InverseKinematics.mot
                  resources/arm26_Reserve_Actuators.xml
                  resources/arm26_Setup_ComputedMuscleControl_EMG.xml
                  resources/muscle_emg.sto
                  resources/std_arm26_states.sto
        )

OpenSimAddTest(NAME testCMCEMGDrivenArm_Thelen
        LINKLIBS osimTools
        RESOURCES resources/arm26.osim
                  resources/arm26_ComputedMuscleControl_Tasks_EMG.xml
                  resources/arm26_InverseKinematics.mot
                  resources/arm26_Reserve_Actuators.xml
                  resources/arm26_Setup_ComputedMuscleControl_EMG.xml
                  resources/muscle_emg.sto
                  resources/std_arm26_states.sto
        )

OpenSimAddTest(NAME testCMCGait10dof18musc
        LINKLIBS osimTools
        RESOURCES resources/gait10dof18musc_Setup_CMC.xml
                  resources/gait10dof18musc_std_walk_subject_states_unix.sto
                  resources/gait10dof18musc_subject01.osim
                  resources/gait10dof18musc_subject01_walk_grf.mot
                  resources/gait10dof18musc_subject01_walk_grf.xml
                  resources/gait10dof18musc_subject_adjusted_Kinematics_q.sto
                  resources/gait10dof_Kinematics_Tracking_Tasks.xml
                  resources/gait10dof_Reserve_Actuators.xml
        )

OpenSimAddTest(NAME testCMCLeg6Dof9Musc
        LINKLIBS osimTools
        RESOURCES resources/Leg6Dof9MuscSwing_std_leg6dof9musc_states.sto
                  resources/leg69_CMC_Swing_Tracking_Tasks.xml
                  resources/leg69_CMC_residuals_motors.xml
                  resources/leg69_IK_stance.mot
                  resources/leg69_IK_swing.mot
                  resources/leg69_Setup_CMC_Stance.xml
                  resources/leg69_Tracking_Tasks.xml
                  resources/leg69_muscles_control_limits.xml
                  resources/leg69_muscles_residuals_motor_control_limits.xml
                  resources/leg69_right_grf.xml
                  resources/leg69_stance_grf.mot
                  resources/leg6dof9musc.osim
                  resources/leg6dof9musc_added_mass.osim
                  resources/leg6dof9musc_adjusted_Kinematics_q.sto
                  resources/leg6dof9musc_setup_Swing.xml
                  resources/leg6dof9musc_std_RRA_adjusted_states.sto
        )

OpenSimAddTest(NAME testCMCSingleMuscle
        LINKLIBS osimTools
        RESOURCES resources/block_hanging_from_muscle.osim
                  resources/block_hanging_from_muscle_CMC_Tasks.xml
                  resources/block_hanging_from_muscle_Setup_CMC.xml
                  resources/block_hanging_from_muscle_Setup_Forward.xml
                  resources/block_hanging_from_muscle_controls.sto
        )

OpenSimAddTest(NAME testCMCSingleRigidTendonMuscle
        LINKLIBS osimTools
        RESOURCES resources/block_hanging_from_muscle.osim
                  resources/block_hanging_from_muscle_CMC_Tasks.xml
                  resources/block_hanging_from_muscle_Setup_CMC.xml
                  resources/block_hanging_from_muscle_Setup_Forward.xml
                  resources/block_hanging_from_muscle_controls.sto
        )

OpenSimAddTest(NAME testCMCTwoMusclesOnBlock
        LINKLIBS osimTools
        RESOURCES resources/anchor1.vtp
                  resources/anchor2.vtp
                  resources/block.vtp
                  resources/twoMusclesOnBlock.osim
                  resources/twoMusclesOnBlock_CMC_Tasks.xml
                  resources/twoMusclesOnBlock_Setup_CMC.xml
                  resources/twoMusclesOnBlock_Setup_Forward.xml
        )

OpenSimAddTest(NAME testCMCWithControlConstraintsGait2354
        LINKLIBS osimTools
        RESOURCES resources/gait2354_CMC_Actuators.xml
                  resources/gait2354_CMC_ControlConstraints.xml
                  resources/gait2354_CMC_Tasks.xml
                  resources/std_subject01_walk1_states_WithControlConstraints.sto
                  resources/subject01_Setup_CMC.xml
                  resources/subject01_externalForces.xml
                  resources/subject01_simbody_adjusted.osim
                  resources/subject01_walk1_RRA_Kinematics_initial_q.sto
                  resources/subject01_walk1_grf.mot
        )

OpenSimAddTest(NAME testCMCWithControlConstraintsRunningModel
        LINKLIBS osimTools
        RESOURCES resources/runningModel_CMC_Actuators.xml
                  resources/runningModel_CMC_Tasks.xml
                  resources/runningModel_CMC_test.osim
                  resources/runningModel_ControlConstraints.xml
                  resources/runningModel_GRF.xml
                  resources/runningModel_GRF_data.mot
                  resources/runningModel_Kinematics_q.sto
                  resources/runningModel_Setup_CMC_test.xml
                  resources/std_runningModel_CMC_states.sto
        )
