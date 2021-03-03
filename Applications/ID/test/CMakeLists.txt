
file(GLOB TEST_PROGS "test*.cpp")
file(GLOB TEST_FILES *.osim *.xml *.sto *.mot *.trc)
list(APPEND TEST_FILES ${CMAKE_SOURCE_DIR}/OpenSim/Tests/shared/ThoracoscapularShoulderModel.osim)

OpenSimAddTests(
    TESTPROGRAMS ${TEST_PROGS}
    DATAFILES ${TEST_FILES}
    LINKLIBS osimTools
    )
