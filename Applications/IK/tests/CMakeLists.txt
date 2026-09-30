find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

OpenSimAddTest(NAME testIK
        LINKLIBS osimTools
        RESOURCES resources/abdbonepin.trc
                  resources/constraintTest.trc
                  resources/constraintTest_ikOnly.osim
                  resources/constraintTest_setup_ik.xml
                  resources/gait2354_IK_Tasks_uniform.xml
                  resources/setup_ik_abdbonepin.xml
                  resources/std_subject01_walk1_ik.mot
                  resources/std_subject1_abdbonepin_IK.mot
                  resources/subject01_Setup_InverseKinematics.xml
                  resources/subject01_Setup_InverseKinematics_NoModel.xml
                  resources/subject01_simbody.osim
                  resources/subject01_synthetic_marker_data.trc
                  resources/subject1_abdbonepin.osim
        )

OpenSimAddTest(NAME testLiveIK
        LINKLIBS osimTools
        RESOURCES resources/std_subject01_walk1_ik.mot
                  resources/subject01_simbody.osim
        )
