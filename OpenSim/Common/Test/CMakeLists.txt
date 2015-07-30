
file(GLOB TEST_PROGS "*est*.cpp")
file(GLOB TEST_FILES *.osim *.xml *.sto *.mot *.trc)

OPENSIM_ADD_TESTS(
    TESTPROGRAMS ${TEST_PROGS}
    DATAFILES ${TEST_FILES}
    LINKLIBS osimTools ${SIMTK_ALL_LIBS}
    )
