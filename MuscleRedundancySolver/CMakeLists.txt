project(MuscleRedundancySolver)
cmake_minimum_required(VERSION 3.2)

set(CMAKE_CXX_STANDARD 11)
find_package(Simbody 3.6 REQUIRED)
find_package(OpenSim 4.0 REQUIRED)

add_subdirectory(MuscleRedundancySolver)
add_subdirectory(tests)