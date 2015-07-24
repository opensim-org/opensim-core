
file(GLOB TEST_PROGS "test*.cpp")
file(GLOB TEST_FILES *.osim *.xml *.sto *.mot)

OPENSIM_ADD_TESTS(
    TESTPROGRAMS ${TEST_PROGS}
    DATAFILES ${TEST_FILES}
    LINKLIBS osimSimulation osimCommon osimAnalyses
    )
