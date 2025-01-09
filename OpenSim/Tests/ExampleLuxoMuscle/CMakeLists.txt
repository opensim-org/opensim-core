# Define test target and current test dir
set(TEST_TARGET testExampleLuxoMuscle)
set(TEST_DIR ${CMAKE_CURRENT_BINARY_DIR})

# Copy dependencies from ExampleLuxoMuscle into the run directory.
set(EXAMPLE_DIR "${OpenSim_SOURCE_DIR}/OpenSim/Examples/ExampleLuxoMuscle")
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

set(TEST_EXAMPLE_DIR "${OpenSim_SOURCE_DIR}/OpenSim/Tests/ExampleLuxoMuscle")

# Add test.
file(GLOB SOURCE_FILES ${EXAMPLE_DIR}/*.cpp
                       ${EXAMPLE_DIR}/*.h)
add_executable(${TEST_TARGET} ${SOURCE_FILES})
target_link_libraries(${TEST_TARGET} osimTools)
add_test(NAME ${TEST_TARGET}
         COMMAND ${TEST_TARGET} noVisualizer)
