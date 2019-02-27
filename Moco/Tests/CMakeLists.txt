include(CMakeParseArguments)

# Create an exectuable for the file ${TEST_NAME}.cpp, which depends on
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
    target_link_libraries(${MOCOTEST_NAME} osimMoco ${MOCOTEST_LIB_DEPENDS})
    target_include_directories(${MOCOTEST_NAME}
            PRIVATE "${CMAKE_SOURCE_DIR}/tropter/external/catch")
    add_test(NAME ${MOCOTEST_NAME} COMMAND ${MOCOTEST_NAME})
    # Tests likely run faster if we parallelize the tests rather than the
    # individual problems.
    set_property(TEST ${MOCOTEST_NAME} APPEND PROPERTY
            ENVIRONMENT "OPENSIM_MOCO_PARALLEL=0")
    file(COPY ${MOCOTEST_RESOURCES} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
endfunction()

MocoAddTest(NAME testMocoInterface)

MocoAddTest(NAME testMocoCosts)

MocoAddTest(NAME testMocoParameters)

MocoAddTest(NAME testImplicit)

MocoAddTest(NAME testConstraints)

MocoAddTest(NAME testContact)

MocoAddTest(NAME testHangingMassRoundtrip)

MocoAddTest(NAME testSingleMuscle)

MocoAddTest(NAME testSingleMuscleDeGrooteFregly2016 LIB_DEPENDS tropter)

MocoAddTest(NAME testSingleMuscleDeGrooteFregly2016MomentArms
        LIB_DEPENDS tropter)

MocoAddTest(NAME testTugOfWarDeGrooteFregly2016 LIB_DEPENDS tropter
        RESOURCES
        testTugOfWarDeGrooteFregly2016_INDYGO_initial_guess.csv)

MocoAddTest(NAME test2Muscles2DOFsDeGrooteFregly2016 LIB_DEPENDS tropter
        RESOURCES
        test2Muscles2DOFsDeGrooteFregly2016_INDYGO_initial_guess.csv
        test2Muscles2DOFsDeGrooteFregly2016_GSO_setup.xml
        test2Muscles2DOFsDeGrooteFregly2016_INDYGO_setup.xml)

MocoAddTest(NAME test2Muscles1DOFDeGrooteFregly2016 LIB_DEPENDS tropter)
        
MocoAddTest(NAME testGait10dof18musc RESOURCES
        testGait10dof18musc_subject01.osim
        testGait10dof18musc_kinematics.mot
        testGait10dof18musc_netgeneralizedforces.mot
        testGait10dof18musc_GSO_setup.xml
        testGait10dof18musc_INDYGO_setup.xml
        std_testGait10dof18musc_INDYGO_solution_activation.sto
        std_testGait10dof18musc_INDYGO_solution_excitation.sto
        std_testGait10dof18musc_INDYGO_solution_norm_fiber_length.sto
        std_testGait10dof18musc_INDYGO_solution_norm_fiber_velocity.sto
        std_testGait10dof18musc_INDYGO_solution_other_controls.sto
        std_testGait10dof18musc_INDYGO_solution_tendon_force.sto
        std_testGait10dof18musc_INDYGO_solution_norm_tendon_force.sto
        std_testGait10dof18musc_INDYGO_solution_tendon_force_rate_control.sto
        std_testGait10dof18musc_GSO_solution_activation.sto
        std_testGait10dof18musc_GSO_solution_norm_fiber_length.sto
        std_testGait10dof18musc_GSO_solution_norm_fiber_velocity.sto
        std_testGait10dof18musc_GSO_solution_other_controls.sto
        std_testGait10dof18musc_GSO_solution_tendon_force.sto
        std_testGait10dof18musc_GSO_solution_norm_tendon_force.sto
        )
