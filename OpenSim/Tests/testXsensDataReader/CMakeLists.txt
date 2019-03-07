set(TEST_XSENS_READER testXsensDataReader)    

add_executable(${TEST_XSENS_READER} testXsensDataReader.cpp)

target_link_libraries(${TEST_XSENS_READER} osimCommon)

