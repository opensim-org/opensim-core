# Define test target and current test dir
set(TEST_TARGET testAddComponents)
set(TEST_DIR ${CMAKE_CURRENT_BINARY_DIR})

# Copy dependencies from ExampleMain into the run directory
set(EXAMPLE_DIR ${OpenSim_SOURCE_DIR}/OpenSim/Examples/ExampleMain)
file(GLOB TEST_FILES 
    ${EXAMPLE_DIR}/*.obj 
    ${EXAMPLE_DIR}/*.osim 
    ${EXAMPLE_DIR}/*.xml 
    ${EXAMPLE_DIR}/*.sto 
    ${EXAMPLE_DIR}/*.mot
    *.obj *.osim *.xml *.sto *.mot)

#
# Copy example files to run dir
#
foreach(dataFile ${TEST_FILES})
    file(COPY ${dataFile} DESTINATION ${TEST_DIR})
endforeach(dataFile) 

# Copy the standard answer for example main
set(TEST_EXAMPLE_DIR ${OpenSim_SOURCE_DIR}/OpenSim/Tests/ExampleMain)
file(GLOB STD_TEST_FILES 
    ${TEST_EXAMPLE_DIR}/*.sto 
    ${TEST_EXAMPLE_DIR}/*.mot)
    
#
# Copy standard test results to the run dir
#
foreach(dataFile ${STD_TEST_FILES})
    file(COPY ${dataFile} DESTINATION ${TEST_DIR})
endforeach(dataFile)     
    
#
# Testing
#
OpenSimAddTests(
    TESTPROGRAMS ${TEST_TARGET}.cpp
    LINKLIBS osimTools osimSimulation osimActuators
    )