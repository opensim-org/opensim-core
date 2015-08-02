file(GLOB TEST_PROGS "test*.cpp")

OpenSimCopySharedTestFiles(arm26.osim)

OPENSIM_ADD_TESTS(
    TESTPROGRAMS ${TEST_PROGS}
    LINKLIBS osimSimulation osimActuators
    )
