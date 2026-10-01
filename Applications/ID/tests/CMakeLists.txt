find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

OpenSimAddTest(NAME testID
        LINKLIBS osimTools
        RESOURCES "${CMAKE_SOURCE_DIR}/OpenSim/Tests/shared/ThoracoscapularShoulderModel.osim"
                  resources/ThorascapularShoulderModel_ID_static.xml
                  resources/ThorascapularShoulderModel_static.mot
                  resources/arm26.osim
                  resources/arm26_InverseKinematics.mot
                  resources/arm26_Setup_InverseDynamics.xml
                  resources/std_arm26_InverseDynamics.sto
                  resources/std_subject01_InverseDynamics.sto
                  resources/subject01.osim
                  resources/subject01_Setup_InverseDynamics.xml
                  resources/subject01_walk1_grf.mot
                  resources/subject01_walk1_grf.xml
                  resources/subject01_walk1_ik.mot
        )
