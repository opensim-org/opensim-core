
file(GLOB TEST_PROGS "*est*.cpp")
file(GLOB TEST_FILES *.osim *.xml *.sto *.mot *.trc)

OpenSimAddTests(
    TESTPROGRAMS ${TEST_PROGS}
    DATAFILES ${TEST_FILES}
    LINKLIBS ${BTK_LIBRARIES} osimCommon ${SIMTK_ALL_LIBS} 
    )
