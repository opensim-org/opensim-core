add_executable(exampleTRCFileAdapter exampleTRCFileAdapter.cpp)
target_link_libraries(exampleTRCFileAdapter ${Simbody_LIBRARIES} osimCommon)
set_target_properties(exampleTRCFileAdapter PROPERTIES FOLDER "Examples")
file(COPY ${CMAKE_SOURCE_DIR}/OpenSim/Common/Test/dataWithNaNsOfDifferentCases.trc
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_executable(exampleMOTFileAdapter exampleMOTFileAdapter.cpp)
target_link_libraries(exampleMOTFileAdapter ${Simbody_LIBRARIES} osimCommon)
set_target_properties(exampleMOTFileAdapter PROPERTIES FOLDER "Examples")
file(COPY ${CMAKE_SOURCE_DIR}/OpenSim/Tests/ExampleMain/std_tugOfWar_forces.mot
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

if(WITH_BTK)
    add_executable(exampleC3DFileAdapter exampleC3DFileAdapter.cpp)
    target_link_libraries(exampleC3DFileAdapter ${Simbody_LIBRARIES} osimCommon)
    set_target_properties(exampleC3DFileAdapter PROPERTIES FOLDER "Examples")
    file(COPY ${CMAKE_SOURCE_DIR}/OpenSim/Tests/shared/singleLeglanding_2.c3d
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()
