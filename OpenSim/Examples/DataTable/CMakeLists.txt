include_directories(${PROJECT_SOURCE_DIR})

add_executable(example1DataTable example1.cpp)
target_link_libraries(example1DataTable ${Simbody_LIBRARIES} osimCommon)

add_executable(example2DataTable example2.cpp)
target_link_libraries(example2DataTable ${Simbody_LIBRARIES} osimCommon)
