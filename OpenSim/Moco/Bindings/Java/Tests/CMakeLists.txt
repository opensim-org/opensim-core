# See towards the bottom of the file for adding new test files.

include(UseJava)

macro(MocoAddJavaTest TESTNAME)
    get_filename_component(_full_path_to_file "${TESTNAME}.java" ABSOLUTE)

    # For compiling the test.
    # TODO with MSVC and Xcode we should try using per-configuration
    # output directories.
    add_jar(Java_${TESTNAME} SOURCES "${_full_path_to_file}"
        INCLUDE_JARS "${SWIG_JAVA_JAR_BUILD_OUTPUT_PATH}"
        OUTPUT_NAME "${TESTNAME}"
        OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/${TESTNAME}"
        )
        
    set_target_properties(Java_${TESTNAME} PROPERTIES
        PROJECT_LABEL "Compile - Java_${TESTNAME}" 
        FOLDER "Moco/Bindings")

    add_dependencies(Java_${TESTNAME} MocoJavaBindings)

    set(_deps_for_RunMocoJavaTests ${_deps_for_RunMocoJavaTests} Java_${TESTNAME})

    # For running the test.
    if(WIN32)
        set(_sep ";")
    else()
        set(_sep ":")
    endif()
    if(MSVC OR XCODE)
        foreach(cfg ${CMAKE_CONFIGURATION_TYPES})
            add_test(NAME Java_${TESTNAME}_${cfg}
                COMMAND ${Java_JAVA_EXECUTABLE}
                        -enableassertions
                        "-Djava.library.path=${CMAKE_BINARY_DIR}/${cfg}${_sep}${OpenSim_LIB_DIR}"
                        -classpath "${TESTNAME}.jar${_sep}${SWIG_JAVA_JAR_BUILD_OUTPUT_PATH}"
                        ${TESTNAME}
                WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${TESTNAME}"
                CONFIGURATIONS ${cfg}
                )
        endforeach()
    else()
        add_test(NAME Java_${TESTNAME}
            COMMAND ${Java_JAVA_EXECUTABLE}
                    -enableassertions
                    "-Djava.library.path=${CMAKE_BINARY_DIR}/${cfg}${_sep}${OpenSim_LIB_DIR}"
                    -classpath "${TESTNAME}.jar${_sep}${SWIG_JAVA_JAR_BUILD_OUTPUT_PATH}"
                    ${TESTNAME}
            WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${TESTNAME}"
            )
    endif()

    if(WIN32)
        # On Windows, CMake cannot use RPATH to hard code the location of
        # libraries in the binary directory (DLL's don't have RPATH), so we
        # must set PATH to find the right libraries. The location of the
        # libraries depends on the build configuration. We also don't want to
        # accidentally use a different OpenSim build/installation somewhere on
        # the machine.
        foreach(cfg ${CMAKE_CONFIGURATION_TYPES})
            set_tests_properties(Java_${TESTNAME}_${cfg} PROPERTIES ENVIRONMENT
                "PATH=${CMAKE_BINARY_DIR}/${cfg}"
                TIMEOUT 30)
            # Timeout (seconds) is because Java might hang if it can't find
            # the DLL's.
        endforeach()
    endif()

endmacro()

# Allow MSVC users to run only the Java tests directly from the MSVC GUI.
# The Java tests are run from RUN_TESTS, so no need to run this target as
# part of `ALL_BUILD` (e.g, in MSVC).
add_custom_target(RunMocoJavaTests
    COMMAND ${CMAKE_CTEST_COMMAND} --tests-regex Java
                                   --build-config ${CMAKE_CFG_INTDIR}
                                   --extra-verbose
    )

set_target_properties(RunMocoJavaTests PROPERTIES
    PROJECT_LABEL "Java - run tests" FOLDER "Moco/Bindings")


# Specify the tests here!
# -----------------------
# Test name must be the name of a java file in this directory.
# Leave out the ".java" extension.
MocoAddJavaTest(TestSlidingMass)

# This line must appear after all tests are added.
# This variable at the end has been accumulated from within the
# MocoAddJavaTest macro.
add_dependencies(RunMocoJavaTests ${_deps_for_RunMocoJavaTests})




