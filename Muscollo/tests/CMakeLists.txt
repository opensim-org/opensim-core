include(CMakeParseArguments)

# Create an exectuable for the file ${TEST_NAME}.cpp, which depends on
# libraries ${LIB_DEPENDENCIES}. Also create a CTest test for this executable.
function(MuscolloAddTest)
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs LIB_DEPENDS RESOURCES)
    cmake_parse_arguments(MUCOTEST
            "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    add_executable(${MUCOTEST_NAME} ${MUCOTEST_NAME}.cpp testing.h)
    # To organize targets in Visual Studio's Solution Explorer.
    set_target_properties(${MUCOTEST_NAME} PROPERTIES FOLDER "Muscollo/Tests")
        message("DEBUG ${MUCOTEST_NAME} ${MUCOTEST_LIB_DEPENDS}")
    target_link_libraries(${MUCOTEST_NAME} osimMuscollo ${MUCOTEST_LIB_DEPENDS})
    add_test(NAME ${MUCOTEST_NAME} COMMAND ${MUCOTEST_NAME})
    #if(WIN32) # Instead, we are copying dependencies' DLLs.
    #    set_property(TEST ${MUCOTEST_NAME} APPEND PROPERTY
    #        ENVIRONMENT "PATH=${IPOPT_DIR}/bin\;${ADOLC_DIR}/bin\;${OpenSim_ROOT_DIR}/bin")
    #endif()
    file(COPY ${MUCOTEST_RESOURCES} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
endfunction()

MuscolloAddTest(NAME testHangingMassRoundtrip)

MuscolloAddTest(NAME testSingleMuscle)

MuscolloAddTest(NAME testSingleMuscleDeGrooteFregly2016 LIB_DEPENDS tropter)

MuscolloAddTest(NAME testSingleMuscleDeGrooteFregly2016MomentArms
        LIB_DEPENDS tropter)

MuscolloAddTest(NAME testTugOfWarDeGrooteFregly2016 LIB_DEPENDS tropter
        RESOURCES
        testTugOfWarDeGrooteFregly2016_INDYGO_initial_guess.csv)

MuscolloAddTest(NAME test2Muscles2DOFsDeGrooteFregly2016 LIB_DEPENDS tropter
        RESOURCES
        test2Muscles2DOFsDeGrooteFregly2016_INDYGO_initial_guess.csv
        test2Muscles2DOFsDeGrooteFregly2016_GSO_setup.xml
        test2Muscles2DOFsDeGrooteFregly2016_INDYGO_setup.xml)

MuscolloAddTest(NAME testGait10dof18musc RESOURCES
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
        std_testGait10dof18musc_GSO_solution_activation.sto
        std_testGait10dof18musc_GSO_solution_norm_fiber_length.sto
        std_testGait10dof18musc_GSO_solution_norm_fiber_velocity.sto
        std_testGait10dof18musc_GSO_solution_other_controls.sto
        std_testGait10dof18musc_GSO_solution_tendon_force.sto)
