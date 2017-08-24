# Create an exectuable for the file ${TEST_NAME}.cpp, which depends on
# libraries ${LIB_DEPENDENCIES}. Also create a CTest test for this executable.
function(MuscolloAddTest TEST_NAME LIB_DEPENDENCIES)
    add_executable(${TEST_NAME} ${TEST_NAME}.cpp testing.h)
    # To organize targets in Visual Studio's Solution Explorer.
    set_target_properties(${TEST_NAME} PROPERTIES FOLDER "Muscollo/Tests")
    target_link_libraries(${TEST_NAME} ${LIB_DEPENDENCIES})
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    #if(WIN32) # Instead, we are copying dependencies' DLLs.
    #    set_property(TEST ${TEST_NAME} APPEND PROPERTY
    #        ENVIRONMENT "PATH=${IPOPT_DIR}/bin\;${ADOLC_DIR}/bin\;${OpenSim_ROOT_DIR}/bin")
    #endif()
endfunction()

MuscolloAddTest(testHangingMassRoundtrip osimMuscollo)

MuscolloAddTest(testSingleMuscle osimMuscollo)

MuscolloAddTest(testSingleMuscleDeGrooteFregly2016 osimMuscollo)

MuscolloAddTest(testSingleMuscleDeGrooteFregly2016MomentArms
        osimMuscollo)

MuscolloAddTest(testTugOfWarDeGrooteFregly2016 osimMuscollo)
file(COPY testTugOfWarDeGrooteFregly2016_INDYGO_initial_guess.csv
        DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

MuscolloAddTest(test2Muscles2DOFsDeGrooteFregly2016 osimMuscollo)
file(COPY test2Muscles2DOFsDeGrooteFregly2016_INDYGO_initial_guess.csv
          test2Muscles2DOFsDeGrooteFregly2016_GSO_setup.xml
        test2Muscles2DOFsDeGrooteFregly2016_INDYGO_setup.xml
        DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

MuscolloAddTest(testGait10dof18musc osimMuscollo)
file(COPY testGait10dof18musc_subject01.osim
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
          std_testGait10dof18musc_GSO_solution_tendon_force.sto
        DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
