
file(GLOB_RECURSE TEST_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" *.osim)

OpenSimAddExampleCXX(NAME ExampleAbsoluteCoordinates SUBDIR TaskSpace
    EXECUTABLES ExampleAbsoluteCoordinates
    RESOURCES "${TEST_FILES}")

# add_executable(ExampleAbsoluteCoordinates ExampleAbsoluteCoordinates.cpp)

# message(STATUS "OpenSim LIBRARIES: ${OpenSim_LIBRARIES}")

# target_link_libraries(ExampleAbsoluteCoordinates PRIVATE osimTaskSpace)

# message(STATUS "OpenSim INCLUDE DIRS: ${OpenSim_INCLUDE_DIRS}")

# target_include_directories(
#     ExampleAbsoluteCoordinates 
#     PRIVATE 
#     "${CMAKE_SOURCE_DIR}/src"
#     ${OpenSim_INCLUDE_DIRS}
#     "${OpenSim_INCLUDE_DIRS}/OpenSim"
#     ${Simbody_INCLUDE_DIR}
#     "${OpenSim_INCLUDE_DIRS}/../spdlog/include")