# Create an exectuable for the file ${TEST_NAME}.cpp, which depends on
# libraries ${LIB_DEPENDENCIES}. Also create a CTest test for this executable.
function(MRSAddTest TEST_NAME LIB_DEPENDENCIES)
    add_executable(${TEST_NAME} ${TEST_NAME}.cpp)
    target_link_libraries(${TEST_NAME} ${LIB_DEPENDENCIES})
    # SYSTEM avoids warnings from dependencies.
    target_include_directories(${TEST_NAME}
            SYSTEM PRIVATE ${OpenSim_INCLUDE_DIRS})
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

MRSAddTest(testHangingMassRoundtrip osimMuscleRedundancySolver)
MRSAddTest(testSingleMuscle osimMuscleRedundancySolver)
