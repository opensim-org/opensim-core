include_directories(${PROJECT_SOURCE_DIR})
add_executable(exampleDataTable example.cpp)
target_link_libraries(exampleDataTable ${Simbody_LIBRARIES} osimCommon)
