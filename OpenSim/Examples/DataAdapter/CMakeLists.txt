include_directories(${PROJECT_SOURCE_DIR})

add_executable(exampleTRCFileAdapter exampleTRCFileAdapter.cpp)
target_link_libraries(exampleTRCFileAdapter ${Simbody_LIBRARIES} osimCommon)

add_executable(exampleMOTFileAdapter exampleMOTFileAdapter.cpp)
target_link_libraries(exampleMOTFileAdapter ${Simbody_LIBRARIES} osimCommon)
