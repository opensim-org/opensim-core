file(GLOB TEST_PROGS "test*.cpp")
file(GLOB TEST_FILES resources/*.osim
                     resources/*.xml
                     resources/*.sto
                     resources/*.mot
                     resources/*.trc)

OpenSimAddTests(
    TESTPROGRAMS ${TEST_PROGS}
    DATAFILES ${TEST_FILES}
    LINKLIBS osimTools
    )
