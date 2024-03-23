file(GLOB_RECURSE TEST_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" *.osim)

OpenSimAddExampleCXX(NAME ExampleTaskBasedControlSUBDIR TaskSpace
    EXECUTABLES ExampleTaskBasedControl
    RESOURCES "${TEST_FILES}")