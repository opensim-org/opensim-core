if(BUILD_JAVA_WRAPPING) 

    file(GLOB TEST_PROGS "test*.cpp")
    file(GLOB TEST_FILES *.osim *.xml *.sto *.mot)

    OpenSimAddTests(
        TESTPROGRAMS ${TEST_PROGS}
        DATAFILES ${TEST_FILES}
        LINKLIBS osimJavaJNI
        SOURCES "../OpenSimContext.cpp"
        )

endif(BUILD_JAVA_WRAPPING)
