find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

file(GLOB TEST_PROGS "*est*.cpp")

OpenSimAddTests(
    TESTPROGRAMS ${TEST_PROGS}
    LINKLIBS osimLepton Catch2::Catch2WithMain
    )
