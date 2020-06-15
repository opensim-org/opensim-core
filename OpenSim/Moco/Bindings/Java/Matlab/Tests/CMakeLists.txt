# See the bottom of the file for adding new test files.

# How to set java.library.path for MATLAB:
# http://www.mathworks.com/matlabcentral/answers/102751-how-do-i-configure-the-java-run-time-library-path-java-library-path-in-matlab-with-and-without-adm

set(matlab_output_dir "${CMAKE_BINARY_DIR}/Matlab")
file(WRITE "${matlab_output_dir}/javaclasspath.txt"
    "${SWIG_JAVA_JAR_BUILD_OUTPUT_PATH}")
set(_binary_dirs "${OpenSim_LIB_DIR}")
if(MSVC OR XCODE) 
    # TODO: should not write all configuration build dirs to the
    # librarypath.txt file; this means that Debug libraries are available to
    # the Release build, for example.
    foreach(cfg ${CMAKE_CONFIGURATION_TYPES})
        set(_binary_dirs "${CMAKE_BINARY_DIR}/${cfg}\n${_binary_dirs}")
    endforeach()
else() # Single-configuration generator.
    set(_binary_dirs "${CMAKE_BINARY_DIR}\n${_binary_dirs}")
endif()
file(WRITE "${matlab_output_dir}/javalibrarypath.txt"
    "${_binary_dirs}"
    )

function(MocoAddMatlabTest TESTNAME MFILE)
    get_filename_component(_full_path_to_mfile "${MFILE}" ABSOLUTE)
    if(MSVC OR XCODE)
        # Multi-generator configurations are handled differently.
        # TODO Ideally, we would use a different WORKING_DIRECTORY for each
        # config.
        foreach(cfg ${CMAKE_CONFIGURATION_TYPES})
            matlab_add_unit_test(NAME Matlab_${TESTNAME}_${cfg}
                UNITTEST_FILE "${_full_path_to_mfile}"
                TEST_ARGS CONFIGURATIONS ${cfg}
                # ADDITIONAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../Utilities"
                NO_UNITTEST_FRAMEWORK
                )
            #UNITTEST_PRECOMMAND "cd('${matlab_output_dir}')"
            set_tests_properties(Matlab_${TESTNAME}_${cfg} PROPERTIES
                ENVIRONMENT "OPENSIM_USE_VISUALIZER=0")
            if(WIN32)
                # Must set PATH so that osimJavaJNI can find osimCommon, etc.
                set_property(TEST Matlab_${TESTNAME}_${cfg} APPEND PROPERTY
                    ENVIRONMENT "PATH=${CMAKE_BINARY_DIR}/${cfg}")
            endif()
        endforeach()
    else()
        matlab_add_unit_test(NAME Matlab_${TESTNAME}
            UNITTEST_FILE "${_full_path_to_mfile}"
            # ADDITIONAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../Utilities"
            NO_UNITTEST_FRAMEWORK
            )
        set_tests_properties(Matlab_${TESTNAME} PROPERTIES
            ENVIRONMENT "OPENSIM_USE_VISUALIZER=0")
        #TEST_ARGS CONFIGURATIONS Release RelWithDebInfo MinSizeRel
    endif()
endfunction()

# Allow MSVC users to run only the Matlab tests directly from the MSVC GUI.
# The Matlab tests are run from RUN_TESTS, so no need to run this target as
# part of `ALL_BUILD` (e.g, in MSVC).
add_custom_target(RunMocoMatlabTests
    COMMAND ${CMAKE_CTEST_COMMAND} --tests-regex Matlab
                                   --build-config ${CMAKE_CFG_INTDIR}
                                   --extra-verbose)
set_target_properties(RunMocoMatlabTests PROPERTIES
    PROJECT_LABEL "Matlab - run tests"
    FOLDER "Moco/Bindings")

add_dependencies(RunMocoMatlabTests MocoJavaBindings)





# Tests.
# ------

# From examples.
MocoAddMatlabTest(SlidingMass ../Examples/exampleSlidingMass.m)
MocoAddMatlabTest(SWIGAdditionalInterface testSWIGAdditionalInterface.m)
MocoAddMatlabTest(Workflow testWorkflow.m)

# Copy resources.
# file(COPY "${OPENSIM_SHARED_TEST_FILES_DIR}/arm26.osim"
#      DESTINATION "${matlab_output_dir}")
