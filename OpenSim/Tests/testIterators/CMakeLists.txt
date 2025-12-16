find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

file(GLOB TEST_PROGS "test*.cpp")

OpenSimCopySharedTestFiles(arm26.osim)

OpenSimAddTests(
    TESTPROGRAMS ${TEST_PROGS}
    LINKLIBS osimSimulation osimActuators Catch2::Catch2WithMain
    )
