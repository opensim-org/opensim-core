# Define test target and current test dir
set(TEST_TARGET testBodyDragExample)

function(OpenSimCopyTestFiles SRC_DIR DST_DIR)
    file(GLOB TEST_FILES 
         ${SRC_DIR}/*.obj 
         ${SRC_DIR}/*.osim 
         ${SRC_DIR}/*.xml 
         ${SRC_DIR}/*.sto 
         ${SRC_DIR}/*.mot
         *.obj *.osim *.xml *.sto *.mot)

    # Copy example files to run dir
    foreach(dataFile ${TEST_FILES})
      file(COPY ${dataFile} DESTINATION ${DST_DIR})
    endforeach(dataFile)     
endfunction(OpenSimCopyTestFiles)

# Build plugin library.
set(EXAMPLE_DIR
  "${OpenSim_SOURCE_DIR}/OpenSim/Examples/Plugins/BodyDragExample")
file(GLOB SOURCE_FILES ${EXAMPLE_DIR}/*.cpp
                       ${EXAMPLE_DIR}/*.h)
add_library(${TEST_TARGET} SHARED ${SOURCE_FILES})
target_link_libraries(${TEST_TARGET} osimTools)
set_target_properties(${TEST_TARGET} PROPERTIES
                      DEFINE_SYMBOL OSIMPLUGIN_EXPORTS)

# Copy dependencies from BodyDragExample into the run directory.
set(TEST_FILES_DIR "${EXAMPLE_DIR}/TestPlugin/Example1_ThreeMasses")
set(TEST_DIR "${CMAKE_CURRENT_BINARY_DIR}/Example1_ThreeMasses")
OpenSimCopyTestFiles(${TEST_FILES_DIR} ${TEST_DIR})
# Add test.
add_test(NAME ${TEST_TARGET}_Example1
         COMMAND opensim-cmd -L $<TARGET_FILE:${TEST_TARGET}> run-tool
         forward.xml
         WORKING_DIRECTORY ${TEST_DIR})


set(TEST_FILES_DIR "${EXAMPLE_DIR}/TestPlugin/Example2_Pendulum_Drag")
set(TEST_DIR "${CMAKE_CURRENT_BINARY_DIR}/Example2_Pendulum_Drag")
OpenSimCopyTestFiles(${TEST_FILES_DIR} ${TEST_DIR})
# Add test.
add_test(NAME ${TEST_TARGET}_Example2
         COMMAND opensim-cmd -L $<TARGET_FILE:${TEST_TARGET}> run-tool
         forward.xml
         WORKING_DIRECTORY ${TEST_DIR})
