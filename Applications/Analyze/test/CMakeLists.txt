
file(GLOB TEST_PROGS "test*.cpp")
file(GLOB TEST_FILES *.osim *.xml *.sto *.mot)

OpenSimAddTests(
    TESTPROGRAMS ${TEST_PROGS}
    DATAFILES ${TEST_FILES}
    LINKLIBS osimTools
    )

if(BUILD_TESTING)
    file(COPY "UsingRelativePaths"
        DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
endif()
