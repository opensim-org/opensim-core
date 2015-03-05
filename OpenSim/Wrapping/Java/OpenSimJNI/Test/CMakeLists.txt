IF(BUILD_JAVA_WRAPPING) 

    FILE(GLOB TEST_PROGS "test*.cpp")
    FILE(GLOB TEST_FILES *.osim *.xml *.sto *.mot)

    OPENSIM_ADD_TESTS(
        TESTPROGRAMS ${TEST_PROGS}
        DATAFILES ${TEST_FILES}
        LINKLIBS osimJavaJNI
        SOURCES "../OpenSimContext.cpp"
        )

ENDIF(BUILD_JAVA_WRAPPING)
