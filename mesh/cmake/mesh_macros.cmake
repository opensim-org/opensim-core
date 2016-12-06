# These CMake functions serve to reduce duplication across CMakeLists.txt files.
include(CMakeParseArguments)

# TODO document
function(mesh_add_test TEST_NAME)
    add_executable(${TEST_NAME} ${TEST_NAME}.cpp)
    target_link_libraries(${TEST_NAME} mesh)
    # TODO MESH shouldn't know that it's in a larger project.
    target_include_directories(${TEST_NAME}
            PRIVATE "${CMAKE_SOURCE_DIR}/mesh/external/catch")
endfunction()
