file(GLOB_RECURSE TEST_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" *.osim)

OpenSimAddExampleCXX(NAME ExampleClosedKinematicChain SUBDIR TaskSpace
    EXECUTABLES ExampleClosedKinematicChain
    RESOURCES "${TEST_FILES}")