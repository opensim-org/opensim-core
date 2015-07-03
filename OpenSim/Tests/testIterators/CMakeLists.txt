
file(GLOB TEST_PROGS "test*.cpp")
file(GLOB TEST_FILES *.obj *.osim *.xml *.sto *.mot)

OPENSIM_ADD_TESTS(
    TESTPROGRAMS ${TEST_PROGS}
    DATAFILES ${TEST_FILES}
    LINKLIBS osimSimulation osimActuators
    )
