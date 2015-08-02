file(GLOB TEST_PROGS "test*.cpp")
OpenSimCopySharedTestFiles(gait10dof18musc_subject01.osim)

OPENSIM_ADD_TESTS(
    TESTPROGRAMS ${TEST_PROGS}
    LINKLIBS osimTools
    )
