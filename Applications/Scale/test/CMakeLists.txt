
FILE(GLOB TEST_PROGS "test*.cpp")
FILE(GLOB TEST_FILES *.osim *.xml *.sto *.mot *.trc)

OPENSIM_ADD_TESTS(
    TESTPROGRAMS ${TEST_PROGS}
    DATAFILES ${TEST_FILES}
    LINKLIBS osimTools
    )
