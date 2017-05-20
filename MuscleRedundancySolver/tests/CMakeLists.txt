# Create an exectuable for the file ${TEST_NAME}.cpp, which depends on
# libraries ${LIB_DEPENDENCIES}. Also create a CTest test for this executable.
function(MRSAddTest TEST_NAME LIB_DEPENDENCIES)
    add_executable(${TEST_NAME} ${TEST_NAME}.cpp testing.h)
    target_link_libraries(${TEST_NAME} ${LIB_DEPENDENCIES})
    # SYSTEM avoids warnings from dependencies.
    target_include_directories(${TEST_NAME}
            SYSTEM PRIVATE ${OpenSim_INCLUDE_DIRS})
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

