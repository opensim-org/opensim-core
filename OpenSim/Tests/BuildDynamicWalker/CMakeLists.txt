add_executable(testBuildDynamicWalker
    "${OpenSim_SOURCE_DIR}/OpenSim/Examples/BuildDynamicWalker/BuildDynamicWalkerModel.cpp")
target_link_libraries(testBuildDynamicWalker osimTools)
add_test(NAME testBuildDynamicWalker 
         COMMAND testBuildDynamicWalker)
