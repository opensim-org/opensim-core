include(CMakeParseArguments)

find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

# Create an executable for the file ${TEST_NAME}.cpp, which depends on
# libraries ${LIB_DEPENDENCIES}. Also create a CTest test for this executable.
function(MocoAddTest)
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs LIB_DEPENDS RESOURCES)
    cmake_parse_arguments(MOCOTEST
            "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    add_executable(${MOCOTEST_NAME} ${MOCOTEST_NAME}.cpp Testing.h)
    # To organize targets in Visual Studio's Solution Explorer.
    set_target_properties(${MOCOTEST_NAME} PROPERTIES FOLDER "Moco/Tests")
    target_link_libraries(${MOCOTEST_NAME} osimMoco ${MOCOTEST_LIB_DEPENDS} Catch2::Catch2WithMain)
    # Skip tests relying on MocoCasADiSolver if CasADi is not available.
    if(NOT OPENSIM_WITH_CASADI)
        list(APPEND test_args "~*MocoCasADiSolver*")
        list(APPEND test_args "~[casadi]")
    endif()
    if(NOT OPENSIM_WITH_TROPTER)
        list(APPEND test_args "~*MocoTropterSolver*")
        list(APPEND test_args "~[tropter]")
    endif()
    set(test_args "")
    if(APPLE)
        list(APPEND test_args "~[win]~[linux]~[win/linux]~[linux/win]")
    endif()
    if(LINUX)
        list(APPEND test_args "~[win]~[mac]~[win/mac]~[mac/win]")
    endif()
    if(WIN32)
        list(APPEND test_args "~[mac]~[linux]~[mac/linux]~[linux/mac]~[unix]")
    endif()
    add_test(NAME ${MOCOTEST_NAME} COMMAND ${MOCOTEST_NAME} ${test_args})
    # Tests likely run faster if we parallelize the tests rather than the
    # individual problems.
    set_property(TEST ${MOCOTEST_NAME} APPEND PROPERTY
            ENVIRONMENT "OPENSIM_MOCO_PARALLEL=0")
    file(COPY ${MOCOTEST_RESOURCES} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
endfunction()

MocoAddTest(NAME testMocoInterface)

MocoAddTest(NAME testMocoGoals)

MocoAddTest(NAME testMocoParameters)

MocoAddTest(NAME testMocoImplicit)

MocoAddTest(NAME testMocoConstraints)

MocoAddTest(NAME testMocoContact)

MocoAddTest(NAME testMocoActuators)

MocoAddTest(NAME testMocoInverse RESOURCES
        subject_walk_armless_18musc.osim
        subject_walk_armless_coordinates.mot
        subject_walk_armless_grfs.mot
        subject_walk_armless_external_loads.xml
        std_testMocoInverse_subject_18musc_solution.sto
        )

MocoAddTest(NAME testMocoTrack RESOURCES
        walk_gait1018_subject01.osim
        walk_gait1018_state_reference.mot
        walk_gait1018_subject01_grf.xml
        walk_gait1018_subject01_grf.mot
        std_testMocoTrackGait10dof18musc_solution.sto
        )

MocoAddTest(NAME testMocoAnalytic)

MocoAddTest(NAME testMocoMetabolics)

file(COPY subject_walk_armless_18musc.osim subject_walk_armless_coordinates.mot
     DESTINATION "${CMAKE_BINARY_DIR}/OpenSim/Tests/Wrapping")

file(COPY walk_gait1018_subject01.osim walk_gait1018_state_reference.mot
        DESTINATION "${CMAKE_BINARY_DIR}/OpenSim/Tests/Wrapping")
