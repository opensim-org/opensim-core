find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

OpenSimAddTest(NAME testAnalyzeTool
        LINKLIBS osimTools
        RESOURCES resources/BothLegs.osim
                  resources/PlotterTool.xml
                  resources/Tug_of_War_ConstantVelocity.sto
                  resources/Tug_of_War_ConstantVelocity_RampActivation.sto
                  resources/Tug_of_War_Millard_Iso.osim
                  resources/Tug_of_War_Setup_Analyze.xml
                  resources/double_pendulum.osim
                  resources/double_pendum1sec.sto
                  resources/plotterGeneratedStates.sto
                  resources/plotterGeneratedStatesHip45.sto
                  resources/std_BothLegsHip45__FiberLength.sto
                  resources/std_BothLegs_fiberLength.sto
        )

OpenSimAddTest(NAME testInducedAccelerations
        LINKLIBS osimTools
        RESOURCES resources/double_pendulum.osim
                  resources/double_pendulum_Setup_IAA.xml
                  resources/double_pendulum_states.sto
                  resources/gait2392_CMC_Actuators_232.xml
                  resources/pendulum_controls.xml
                  resources/std_subject02_running_arms_InducedAccelerations_CENTER_OF_MASS.sto
                  resources/subject02_Setup_IAA_02_232.xml
                  resources/subject02_grf_HiFreq.mot
                  resources/subject02_grf_HiFreq.xml
                  resources/subject02_running_CMC_controls.xml
                  resources/subject02_running_CMC_states.sto
                  resources/subject02_running_RRA_cycle02_sim02_07_v232.osim
        )

OpenSimAddTest(NAME testJointReactions
        LINKLIBS osimTools
        RESOURCES resources/DoublePendulum3D.osim
                  resources/DoublePendulum3D_Setup_JointReaction.xml
                  resources/DoublePendulum3D_states_degrees.mot
                  resources/SinglePin.osim
                  resources/SinglePin_Setup_JointReaction.xml
                  resources/SinglePin_Setup_JointReaction_FrameKeyword.xml
                  resources/SinglePin_states.sto
                  resources/std_DoublePendulum3D_JointReaction_ReactionLoads.sto
                  resources/std_DoublePendulum3D_JointReaction_ReactionLoads_FrameKeyword.sto
                  resources/std_SinglePin_JointReaction_ReactionLoads.sto
                  resources/std_SinglePin_JointReaction_ReactionLoads_FrameKeyword.sto
        )

OpenSimAddTest(NAME testPointKinematics
        LINKLIBS osimTools
        RESOURCES resources/SinglePin.osim
                  resources/subject01.osim
                  resources/subject01_walk1_ik.mot
        )

OpenSimAddTest(NAME testStaticOptimization
        LINKLIBS osimTools
        RESOURCES resources/arm26.osim
                  resources/arm26_InverseKinematics.mot
                  resources/arm26_Setup_StaticOptimization.xml
                  resources/arm26_bounds.osim
                  resources/arm26_bounds_Setup_StaticOptimization.xml
                  resources/externalForces.xml
                  resources/gait10dof_Actuators_weak.xml
                  resources/staticoptimization_spring_Setup.xml
                  resources/std_Millard2012EquilibriumMuscle_arm26_StaticOptimization_activation.sto
                  resources/std_Millard2012EquilibriumMuscle_arm26_StaticOptimization_force.sto
                  resources/std_Millard2012EquilibriumMuscle_arm26_bounds_StaticOptimization_activation.sto
                  resources/std_Millard2012EquilibriumMuscle_arm26_bounds_StaticOptimization_force.sto
                  resources/std_Thelen2003Muscle_Deprecated_arm26_StaticOptimization_activation.sto
                  resources/std_Thelen2003Muscle_Deprecated_arm26_StaticOptimization_force.sto
                  resources/std_Thelen2003Muscle_Deprecated_arm26_bounds_StaticOptimization_activation.sto
                  resources/std_Thelen2003Muscle_Deprecated_arm26_bounds_StaticOptimization_force.sto
                  resources/std_Thelen2003Muscle_arm26_StaticOptimization_activation.sto
                  resources/std_Thelen2003Muscle_arm26_StaticOptimization_force.sto
                  resources/std_Thelen2003Muscle_arm26_bounds_StaticOptimization_activation.sto
                  resources/std_Thelen2003Muscle_arm26_bounds_StaticOptimization_force.sto
                  resources/std_walk_subject01_ankle_spring_StaticOptimization_activation.sto
                  resources/std_walk_subject01_ankle_spring_StaticOptimization_force.sto
                  resources/subject01.osim
                  resources/subject01_Setup_StaticOptimization.xml
                  resources/subject01_adjusted_spring.osim
                  resources/subject01_walk1_grf.mot
                  resources/subject01_walk1_ik.mot
                  resources/walk_subject01_Kinematics_q.sto
        )

file(COPY "resources/UsingRelativePaths" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
