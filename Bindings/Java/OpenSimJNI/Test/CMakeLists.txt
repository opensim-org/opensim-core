if(BUILD_JAVA_WRAPPING)

    find_package(Catch2 REQUIRED
            HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

    OpenSimAddTest(NAME testContext
        LINKLIBS osimJavaJNI
        EXTRA_SOURCES "../OpenSimContext.cpp"
        RESOURCES arm26_20.osim
                  wrist.osim
        )

endif(BUILD_JAVA_WRAPPING)
