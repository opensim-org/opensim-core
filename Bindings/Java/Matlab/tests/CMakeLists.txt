
# See the bottom of the file for adding new test files.

# How to set java.library.path for MATLAB:
# http://www.mathworks.com/matlabcentral/answers/102751-how-do-i-configure-the-java-run-time-library-path-java-library-path-in-matlab-with-and-without-adm

set(matlab_output_dir "${CMAKE_BINARY_DIR}/Matlab")
file(WRITE "${matlab_output_dir}/javaclasspath.txt"
    "${SWIG_JAVA_JAR_BUILD_OUTPUT_PATH}")
if(MSVC OR XCODE) 
    set(_binary_dirs)
    foreach(cfg ${CMAKE_CONFIGURATION_TYPES})
        set(_binary_dirs "${CMAKE_BINARY_DIR}/${cfg}\n${_binary_dirs}")
    endforeach()
else() # Single-configuration generator.
    set(_binary_dirs "${CMAKE_BINARY_DIR}")
endif()
file(WRITE "${matlab_output_dir}/javalibrarypath.txt"
    "${_binary_dirs}"
    )

macro(OpenSimAddMatlabTest TESTNAME MFILE)
    get_filename_component(_full_path_to_mfile "${MFILE}" ABSOLUTE)
    if(WIN32)
        # On Windows, must set ADDITIONAL_PATH to find the opensim
        # libraries.
        set(_additional_path "${CMAKE_BINARY_DIR}/${CMAKE_CFG_INTDIR}")
    endif()
    matlab_add_unit_test(NAME Matlab_${TESTNAME}
        UNITTEST_FILE "${_full_path_to_mfile}"
        TEST_ARGS "CONFIGURATIONS Release RelWithDebInfo MinSizeRel"
        ADDITIONAL_PATH "${_additional_path}"
        NO_UNITTEST_FRAMEWORK
        )
    #UNITTEST_PRECOMMAND "cd('${matlab_output_dir}')"
endmacro()

# Allow MSVC users to run only the Matlab tests directly from the MSVC GUI.
# The Matlab tests are run from RUN_TESTS, so no need to run this target as
# part of `ALL_BUILD` (e.g, in MSVC).
add_custom_target(RunMatlabTests
    COMMAND ${CMAKE_CTEST_COMMAND} --tests-regex Matlab
                                   ${OPENSIM_TEST_BUILD_CONFIG}
                                   --extra-verbose
    PROJECT_LABEL "Matlab - run tests"
    FOLDER "Bindings"
    )

add_dependencies(RunMatlabTests JavaBindings)





# Tests.
# ------

OpenSimAddMatlabTest(TugOfWar ../examples/OpenSimCreateTugOfWarModel.m)
OpenSimAddMatlabTest(ConnectorsInputsOutputs testConnectorsInputsOutputs.m)
OpenSimAddMatlabTest(AccessSubcomponents testAccessSubcomponents.m)
OpenSimAddMatlabTest(VandenBogert2011Muscle testVandenBogert2011Muscle.m)

# Copy resources.
file(COPY "${OPENSIM_SHARED_TEST_FILES_DIR}/arm26.osim"
     DESTINATION "${matlab_output_dir}")
