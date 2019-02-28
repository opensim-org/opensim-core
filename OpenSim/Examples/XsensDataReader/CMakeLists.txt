add_executable(exampleXsensDataReader exampleXsensDataReader.cpp)
target_link_libraries(exampleXsensDataReader ${Simbody_LIBRARIES} osimCommon)
set_target_properties(exampleXsensDataReader PROPERTIES FOLDER "Examples")
file(COPY ${CMAKE_SOURCE_DIR}/OpenSim/Tests/shared/IMUData/Xsens/
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
