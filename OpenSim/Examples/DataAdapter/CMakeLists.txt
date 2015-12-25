include_directories(${PROJECT_SOURCE_DIR})

add_executable(exampleTRCFileAdapter exampleTRCFileAdapter.cpp)
target_link_libraries(exampleTRCFileAdapter ${Simbody_LIBRARIES} osimCommon)
file(COPY ${CMAKE_SOURCE_DIR}/OpenSim/Common/Test/TRCFileWithNANs.trc
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_executable(exampleMOTFileAdapter exampleMOTFileAdapter.cpp)
target_link_libraries(exampleMOTFileAdapter ${Simbody_LIBRARIES} osimCommon)
file(COPY ${CMAKE_SOURCE_DIR}/OpenSim/Tests/ExampleMain/std_tugOfWar_forces.mot
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_executable(exampleC3DFileAdapter exampleC3DFileAdapter.cpp)
target_link_libraries(exampleC3DFileAdapter ${Simbody_LIBRARIES} osimCommon)
file(COPY ${CMAKE_SOURCE_DIR}/OpenSim/Tests/shared/singleLeglanding_2.c3d
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
