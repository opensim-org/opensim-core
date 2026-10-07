find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

OpenSimAddTest(NAME testOutputReporter
        LINKLIBS osimCommon osimSimulation osimAnalyses osimActuators osimLepton)
