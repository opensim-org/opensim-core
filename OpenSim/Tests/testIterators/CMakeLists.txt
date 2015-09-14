file(GLOB TEST_PROGS "test*.cpp")

OpenSimCopySharedTestFiles(arm26.osim)

OpenSimAddTests(
    TESTPROGRAMS ${TEST_PROGS}
    LINKLIBS osimSimulation osimActuators
    )
