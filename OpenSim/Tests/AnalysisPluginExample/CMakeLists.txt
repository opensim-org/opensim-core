# Define test target and current test dir
set(TEST_TARGET testAnalysisPluginExample)
set(TEST_DIR ${CMAKE_CURRENT_BINARY_DIR})

# Copy dependencies from AnalysisPluginExample into the run directory.
set(EXAMPLE_DIR
  "${OpenSim_SOURCE_DIR}/OpenSim/Examples/Plugins/AnalysisPluginExample")
set(TEST_FILES_DIR "${EXAMPLE_DIR}/TestPlugin")
file(GLOB TEST_FILES 
    ${TEST_FILES_DIR}/*.obj 
    ${TEST_FILES_DIR}/*.osim 
    ${TEST_FILES_DIR}/*.xml 
    ${TEST_FILES_DIR}/*.sto 
    ${TEST_FILES_DIR}/*.mot
    *.obj *.osim *.xml *.sto *.mot)

#
# Copy example files to run dir
#
foreach(dataFile ${TEST_FILES})
    file(COPY ${dataFile} DESTINATION ${TEST_DIR})
endforeach(dataFile) 

set(TEST_EXAMPLE_DIR
    "${OpenSim_SOURCE_DIR}/OpenSim/Tests/AnalysisPluginExample")

# Add test.
file(GLOB SOURCE_FILES ${EXAMPLE_DIR}/*.cpp
                       ${EXAMPLE_DIR}/*.h)
add_library(${TEST_TARGET} SHARED ${SOURCE_FILES})
target_link_libraries(${TEST_TARGET} osimTools)
set_target_properties(${TEST_TARGET} PROPERTIES
                      DEFINE_SYMBOL OSIMPLUGIN_EXPORTS)

add_test(NAME ${TEST_TARGET}
         COMMAND opensim-cmd -L $<TARGET_FILE:${TEST_TARGET}> run-tool
                 subject01_Setup_AnalysisPluginTemplate.xml)
