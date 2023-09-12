
set(TEST_PROGS
        testWrapping.cpp
        # testWrapping_Deprecated.cpp
        testWrapCylinder.cpp
        testWrappingAlgorithm.cpp
)
file(GLOB TEST_FILES *.osim *.xml *.sto *.mot)

OpenSimAddTests(
    TESTPROGRAMS ${TEST_PROGS}
    DATAFILES ${TEST_FILES}
    LINKLIBS osimTools
    )
