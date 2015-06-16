
set(TEST_ENVIRONMENT checkEnvironment)       
         
include_directories(${OpenSim_SOURCE_DIR}        
                 ${OpenSim_SOURCE_DIR}/Vendors)          
 
add_executable(${TEST_ENVIRONMENT} ${OpenSim_SOURCE_DIR}/OpenSim/Examples/checkEnvironment/test.cpp)         
target_link_libraries(${TEST_ENVIRONMENT}        
       debug osimSimulation${CMAKE_DEBUG_POSTFIX}    optimized osimSimulation
       debug osimActuators${CMAKE_DEBUG_POSTFIX}     optimized osimActuators
       debug osimCommon${CMAKE_DEBUG_POSTFIX}        optimized osimCommon
       debug osimAnalyses${CMAKE_DEBUG_POSTFIX}      optimized osimAnalyses
       debug osimTools${CMAKE_DEBUG_POSTFIX}         optimized osimTools
       ${Simbody_LIBRARIES}
)        
set_target_properties(${TEST_ENVIRONMENT}        
       PROPERTIES        
       PROJECT_LABEL "Utilities - ${TEST_ENVIRONMENT}"       
)        
         
file(GLOB DATA_FILES *.osim *.xml *.sto *.mot *.vtp *.obj)       
FOREACH (DATA_FILE ${DATA_FILES})        
  add_custom_command(        
       TARGET ${TEST_ENVIRONMENT}        
       COMMAND ${CMAKE_COMMAND}          
       ARGS -E copy          
       ${DATA_FILE}          
       ${OpenSim_BINARY_DIR}/OpenSim/Examples/checkEnvironment       
  )          
ENDFOREACH (DATA_FILE)       
         
if(BUILD_TESTING)        
  set(TEST_EXAMPLE_MODEL testExampleModel)       
         
  add_test(${TEST_EXAMPLE_MODEL}Runs ${EXECUTABLE_OUTPUT_PATH}/${TEST_ENVIRONMENT})          
endif(BUILD_TESTING)         
         
install_targets(/bin ${TEST_ENVIRONMENT})
