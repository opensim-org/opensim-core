find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

OpenSimAddTest(NAME testRRA
        LINKLIBS osimTools
        RESOURCES resources/gait2354_RRA_Actuators.xml
                  resources/gait2354_RRA_ControlConstraints.xml
                  resources/gait2354_RRA_Tasks.xml
                  resources/subject01_RRA_adjusted.osim
                  resources/subject01_Setup_RRA.xml
                  resources/subject01_externalForces.xml
                  resources/subject01_simbody.osim
                  resources/subject01_walk1_RRA_Kinematics_q_standard.sto
                  resources/subject01_walk1_grf.mot
                  resources/subject01_walk1_ik.mot
        )
