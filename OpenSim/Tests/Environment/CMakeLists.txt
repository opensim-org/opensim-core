
set(TEST_ENVIRONMENT checkEnvironment)       
 
add_executable(${TEST_ENVIRONMENT}
    ${OpenSim_SOURCE_DIR}/OpenSim/Examples/checkEnvironment/test.cpp)         
target_link_libraries(${TEST_ENVIRONMENT} osimTools)
set_target_properties(${TEST_ENVIRONMENT}        
       PROPERTIES        
       PROJECT_LABEL "Utilities - ${TEST_ENVIRONMENT}"       
)        
         
file(GLOB DATA_FILES *.osim *.xml *.sto *.mot *.vtp *.obj)       
foreach(DATA_FILE ${DATA_FILES})        
  add_custom_command(        
       TARGET ${TEST_ENVIRONMENT}        
       COMMAND ${CMAKE_COMMAND}          
       ARGS -E copy          
       ${DATA_FILE}          
       ${OpenSim_BINARY_DIR}/OpenSim/Examples/checkEnvironment       
  )          
endforeach(DATA_FILE)       
         
if(BUILD_TESTING)        
  set(TEST_EXAMPLE_MODEL testExampleModel)       
         
  add_test(${TEST_EXAMPLE_MODEL}Runs ${EXECUTABLE_OUTPUT_PATH}/${TEST_ENVIRONMENT})          
endif(BUILD_TESTING)         
         
install(TARGETS ${TEST_ENVIRONMENT} DESTINATION "${CMAKE_INSTALL_BINDIR}")
