find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

OpenSimAddTest(NAME testForward
        LINKLIBS osimTools
        RESOURCES resources/arm26.osim
                  resources/arm26_InitialStates.sto
                  resources/arm26_Setup_Forward.xml
                  resources/arm26_StaticOptimization_controls.xml
                  resources/gait2354_CMC_Actuators.xml
                  resources/pendulum.osim
                  resources/pendulum_Setup_Forward.xml
                  resources/pendulum_ext_gravity_Setup_Forward.xml
                  resources/pendulum_ext_point_in_ground_Setup_Forward.xml
                  resources/pendulum_external_load.sto
                  resources/pendulum_external_load.xml
                  resources/pendulum_external_load_point_in_ground.sto
                  resources/pendulum_external_load_point_in_ground.xml
                  resources/pendulum_no_gravity.osim
                  resources/std_arm26_states.sto
                  resources/std_subject01_walk1_states.sto
                  resources/subject01_GRFs.xml
                  resources/subject01_Setup_Forward.xml
                  resources/subject01_Setup_Forward_Controller.xml
                  resources/subject01_adjusted.osim
                  resources/subject01_walk1_controls.xml
                  resources/subject01_walk1_grf.mot
        )
