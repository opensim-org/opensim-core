include_directories(${PROJECT_SOURCE_DIR})
add_executable(exampleDataAdapter example.cpp)
target_link_libraries(exampleDataAdapter osimCommon)
