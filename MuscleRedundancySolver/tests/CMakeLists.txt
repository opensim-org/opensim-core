# Create an exectuable for the file ${TEST_NAME}.cpp, which depends on
# libraries ${LIB_DEPENDENCIES}. Also create a CTest test for this executable.
function(MRSAddTest TEST_NAME LIB_DEPENDENCIES)
    add_executable(${TEST_NAME} ${TEST_NAME}.cpp testing.h)
    # To organize targets in Visual Studio's Solution Explorer.
    set_target_properties(${TEST_NAME} PROPERTIES FOLDER "MRS Tests")
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
