
# See towards the bottom of the file for adding new test files.

include(UseJava)

macro(OpenSimAddJavaTest TESTNAME)
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
        FOLDER "Bindings")

    add_dependencies(Java_${TESTNAME} JavaBindings)

    # This macro is invoked one folder down, but we want this variable 
    # to be in the scope of this file (which is the parent scope of the tests
    # folder).
    set(_deps_for_RunJavaTests ${_deps_for_RunJavaTests} Java_${TESTNAME})

    # For running the test.
    if(WIN32)
        set(_sep ";")
    else()
        set(_sep ":")
    endif()
    if(MSVC OR XCODE)
        # TODO make sure this works, using xcode. Look at Python for an
        # example.
        foreach(cfg ${CMAKE_CONFIGURATION_TYPES})
            add_test(NAME Java_${TESTNAME}_${cfg}
                COMMAND ${Java_JAVA_EXECUTABLE}
                        -enableassertions
                        -Djava.library.path=${CMAKE_BINARY_DIR}/${cfg}
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
                    -Djava.library.path=${CMAKE_BINARY_DIR}
                    -classpath "${TESTNAME}.jar${_sep}${SWIG_JAVA_JAR_BUILD_OUTPUT_PATH}"
                    ${TESTNAME}
            WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${TESTNAME}"
            )
    endif()

    # Copy all necessary test files.
    string(CONCAT FILETOCOPY
          "${CMAKE_SOURCE_DIR}/OpenSim/Tests/shared/walking5.c3d")
    file(COPY ${FILETOCOPY}
         DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/TestC3DFileAdapter")
    string(CONCAT FILETOCOPY
          "${CMAKE_SOURCE_DIR}/OpenSim/Tests/shared/IMUData/Xsens/MT_012005D6_031-000_00B421AF.txt")
    file(COPY ${FILETOCOPY}
         DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/TestXsensDataReader")
    string(CONCAT FILETOCOPY
           "${CMAKE_SOURCE_DIR}/Bindings/Java/Matlab/examples/testData/"
           "Subject01/IKSetup/subject01_Setup_IK_generic.xml")
    file(COPY ${FILETOCOPY}
         DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/TestTables")
    string(CONCAT FILETOCOPY
           "${CMAKE_SOURCE_DIR}/Bindings/Java/Matlab/examples/testData/"
           "Subject01/MarkerData/walk_free_01.trc")
    file(COPY ${FILETOCOPY}
         DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/TestTables")
    string(CONCAT FILETOCOPY
           "${CMAKE_SOURCE_DIR}/Bindings/Java/Matlab/examples/testData/"
           "Subject01/subject01_gait2392_scaled.osim")
    file(COPY ${FILETOCOPY}
         DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/TestTables")
    string(CONCAT FILETOCOPY
           "${CMAKE_SOURCE_DIR}/OpenSim/Sandbox/MatlabScripts/"
           "double_pendulum_markers.osim")
    file(COPY ${FILETOCOPY}
         DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/TestReporter")
    file(COPY "${OPENSIM_SHARED_TEST_FILES_DIR}/gait10dof18musc_subject01.osim"
                 DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/TestVisualization")
    file(COPY "${OPENSIM_SHARED_TEST_FILES_DIR}/gait10dof18musc_subject01.osim"
                 DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/TestEditProperties")
    file(COPY "${OPENSIM_SHARED_TEST_FILES_DIR}/gait10dof18musc_subject01.osim"
                 DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/TestEditMarkers")
    file(COPY "${OPENSIM_SHARED_TEST_FILES_DIR}/gait10dof18musc_subject01.osim"
                 DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/TestIKInterfaces")
    if(WIN32)
        # On Windows, CMake cannot use RPATH to hard code the location of libraries
        # in the binary directory (DLL's don't have RPATH), so we must set PATH to
        # find the right libraries. The location of the libraries depends on the
        # build configuration, which is filled in for `$<CONFIG>`. We also don't
        # want to accidentally use a different OpenSim build/installation somewhere
        # on the machine.
        # TODO use the commented-out version when moving to CMake 3.0.
        #set_tests_properties(python PROPERTIES ENVIRONMENT
        #    "PATH=${CMAKE_BINARY_DIR}/$<CONFIG>")
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
add_custom_target(RunJavaTests
    COMMAND ${CMAKE_CTEST_COMMAND} --tests-regex Java
                                   ${OPENSIM_TEST_BUILD_CONFIG}
                                   --extra-verbose
    )

set_target_properties(RunJavaTests PROPERTIES
    PROJECT_LABEL "Java - run tests" FOLDER "Bindings")




# Specify the tests here!
# -----------------------
# Test name must be the name of a java file in this directory.
# Leave out the ".java" extension.
OpenSimAddJavaTest(TestVectors)
OpenSimAddJavaTest(TestBasics)
OpenSimAddJavaTest(TestIterators)
OpenSimAddJavaTest(TestVisualization)
OpenSimAddJavaTest(TestEditProperties)
OpenSimAddJavaTest(TestTables)
OpenSimAddJavaTest(TestModelBuilding)
OpenSimAddJavaTest(TestEditMarkers)
OpenSimAddJavaTest(TestIKInterfaces)
if(WITH_EZC3D)
    OpenSimAddJavaTest(TestC3DFileAdapter)
endif()
OpenSimAddJavaTest(TestReporter)
OpenSimAddJavaTest(TestXsensDataReader)

OpenSimAddJavaTest(TestMocoSlidingMass)

# This line must appear after all tests are added.
# This variable at the end has been accumulated from within the
# OpenSimAddJavaTest macro.
add_dependencies(RunJavaTests ${_deps_for_RunJavaTests})




