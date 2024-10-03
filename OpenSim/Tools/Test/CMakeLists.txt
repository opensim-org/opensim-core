find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

file(GLOB TEST_PROGS "test*.cpp")
file(GLOB TEST_FILES *.osim *.xml *.sto *.mot *.txt *.obj *.vtp)

OpenSimAddTests(
    TESTPROGRAMS ${TEST_PROGS}
    DATAFILES ${TEST_FILES}
    LINKLIBS osimTools Catch2::Catch2WithMain
    )


# edge-case: one test ensures that `ExternalLoads` still works - even when it isn't in
#            the working directory of the test runner (#3926)
file(COPY "ExternalLoadsInSubdir" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
