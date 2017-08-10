# These CMake functions serve to reduce duplication across CMakeLists.txt files.

include(CMakeParseArguments)

# Create an exectuable for the file ${TEST_NAME}.cpp, which depends on
# libraries ${LIB_DEPENDENCIES}. Also create a CTest test for this executable.
# We expect that the test cases use the Catch testing framework, and
# therefore we pass some command-line flags that Catch interprets.
function(mesh_add_test TEST_NAME LIB_DEPENDENCIES)
    add_executable(${TEST_NAME} ${TEST_NAME}.cpp)
    # To organize targets in Visual Studio's Solution Explorer.
    set_target_properties(${TEST_NAME} PROPERTIES FOLDER "Mesh Tests")
    target_link_libraries(${TEST_NAME} ${LIB_DEPENDENCIES})
    # TODO MESH shouldn't know that it's in a larger project.
    target_include_directories(${TEST_NAME}
            PRIVATE "${CMAKE_SOURCE_DIR}/mesh/external/catch")
    if(TOMU_WITH_SNOPT)
        target_compile_definitions(${TEST_NAME} PRIVATE TOMU_WITH_SNOPT)
    endif()
    add_test(NAME ${TEST_NAME}
            COMMAND ${TEST_NAME} --use-colour yes --durations yes)
endfunction()
