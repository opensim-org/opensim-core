file(GLOB_RECURSE TEST_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" *.osim)

OpenSimAddExampleCXX(NAME ExamplePerreault2001Experiment SUBDIR TaskSpace
    EXECUTABLES ExamplePerreault2001Experiment
    RESOURCES "${TEST_FILES}")