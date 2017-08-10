# Create an exectuable for the file ${TEST_NAME}.cpp, which depends on
# libraries ${LIB_DEPENDENCIES}. Also create a CTest test for this executable.
function(MRSAddTest TEST_NAME LIB_DEPENDENCIES)
    add_executable(${TEST_NAME} ${TEST_NAME}.cpp testing.h)
    # To organize targets in Visual Studio's Solution Explorer.
    set_target_properties(${TEST_NAME} PROPERTIES FOLDER "MRS Tests")
    target_link_libraries(${TEST_NAME} ${LIB_DEPENDENCIES})
    # SYSTEM avoids warnings from dependencies
    # No longer need to explicitly include OpenSim's headers;
    # this is handled by linking to OpenSim's libraries.
    # TODO target_include_directories(${TEST_NAME}
    # TODO         SYSTEM PRIVATE ${OpenSim_INCLUDE_DIRS})
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

MRSAddTest(testHangingMassRoundtrip osimMuscleRedundancySolver)

MRSAddTest(testSingleMuscle osimMuscleRedundancySolver)

MRSAddTest(testSingleMuscleDeGroote2016 osimMuscleRedundancySolver)

MRSAddTest(testSingleMuscleDeGroote2016MomentArms osimMuscleRedundancySolver)

MRSAddTest(testTugOfWarDeGroote2016 osimMuscleRedundancySolver)
file(COPY testTugOfWarDeGroote2016_MRS_initial_guess.csv
        DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

MRSAddTest(test2Muscles2DOFsDeGroote2016 osimMuscleRedundancySolver)
file(COPY test2Muscles2DOFsDeGroote2016_MRS_initial_guess.csv
          test2Muscles2DOFsDeGroote2016_GSO_setup.xml
          test2Muscles2DOFsDeGroote2016_MRS_setup.xml
        DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

MRSAddTest(testGait10dof18musc osimMuscleRedundancySolver)
file(COPY testGait10dof18musc_subject01.osim
          testGait10dof18musc_kinematics.mot
          testGait10dof18musc_netgeneralizedforces.mot
          testGait10dof18musc_GSO_setup.xml
          testGait10dof18musc_MRS_setup.xml
          std_testGait10dof18musc_MRS_solution_activation.sto
          std_testGait10dof18musc_MRS_solution_excitation.sto
          std_testGait10dof18musc_MRS_solution_norm_fiber_length.sto
          std_testGait10dof18musc_MRS_solution_norm_fiber_velocity.sto
          std_testGait10dof18musc_MRS_solution_other_controls.sto
          std_testGait10dof18musc_MRS_solution_tendon_force.sto
          std_testGait10dof18musc_GSO_solution_activation.sto
          std_testGait10dof18musc_GSO_solution_norm_fiber_length.sto
          std_testGait10dof18musc_GSO_solution_norm_fiber_velocity.sto
          std_testGait10dof18musc_GSO_solution_other_controls.sto
          std_testGait10dof18musc_GSO_solution_tendon_force.sto
        DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
