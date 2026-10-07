find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

OpenSimAddTest(NAME testConstraints
        LINKLIBS osimSimulation osimCommon osimAnalyses
        RESOURCES resources/testJointConstraints.osim
        )

OpenSimAddTest(NAME testJoints
        LINKLIBS osimSimulation osimCommon osimAnalyses
        RESOURCES resources/double_pendulum_testReverse.osim
                  resources/testJointConstraints.osim
        )
