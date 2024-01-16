file(GLOB_RECURSE TEST_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" *.osim *.obj)

message(STATUS ${TEST_FILES})

OpenSimAddExampleCXX(NAME ExampleCycling SUBDIR TaskSpace
    EXECUTABLES ExampleCycling
    RESOURCES "${TEST_FILES}")