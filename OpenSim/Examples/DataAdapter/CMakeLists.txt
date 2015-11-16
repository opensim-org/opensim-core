include_directories(${PROJECT_SOURCE_DIR})

add_executable(exampleTRCFileAdapter exampleTRCFileAdapter.cpp)
target_link_libraries(exampleTRCFileAdapter ${Simbody_LIBRARIES} osimCommon)
