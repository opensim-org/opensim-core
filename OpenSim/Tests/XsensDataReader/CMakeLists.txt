set(XSENS_READER XsensDataReader)    

add_executable(${XSENS_READER} XsensDataReader.cpp)

target_link_libraries(${XSENS_READER} osimCommon)

set_target_properties(${XSENS_READER}        
       PROPERTIES        
       PROJECT_LABEL "Utilities - ${XSENS_READER}"       
)
