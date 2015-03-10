
SET(TEST_ENVIRONMENT checkEnvironment)       
 
ADD_EXECUTABLE(${TEST_ENVIRONMENT}
    ${OpenSim_SOURCE_DIR}/OpenSim/Examples/checkEnvironment/test.cpp)         
TARGET_LINK_LIBRARIES(${TEST_ENVIRONMENT} osimTools)

SET_TARGET_PROPERTIES(${TEST_ENVIRONMENT}        
       PROPERTIES        
       PROJECT_LABEL "Utilities - ${TEST_ENVIRONMENT}"       
)        
         
FILE(GLOB DATA_FILES *.osim *.xml *.sto *.mot *.vtp *.obj)       
FOREACH (DATA_FILE ${DATA_FILES})        
  ADD_CUSTOM_COMMAND(        
       TARGET ${TEST_ENVIRONMENT}        
       COMMAND ${CMAKE_COMMAND}          
       ARGS -E copy          
       ${DATA_FILE}          
       ${OpenSim_BINARY_DIR}/OpenSim/Examples/checkEnvironment       
  )          
ENDFOREACH (DATA_FILE)       
         
IF(BUILD_TESTING)        
  SET(TEST_EXAMPLE_MODEL testExampleModel)       
         
  ADD_TEST(${TEST_EXAMPLE_MODEL}Runs ${EXECUTABLE_OUTPUT_PATH}/${TEST_ENVIRONMENT})          
ENDIF(BUILD_TESTING)         
         
INSTALL_TARGETS(/bin ${TEST_ENVIRONMENT})
