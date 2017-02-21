project(MuscleRedundancySolver)
cmake_minimum_required(VERSION 3.2)

set(CMAKE_CXX_STANDARD 11)
find_package(Simbody 3.6 REQUIRED)
find_package(OpenSim 4.0 REQUIRED)

add_executable(opensim-mrs opensim-mrs.cpp
        MuscleRedundancySolver.h MuscleRedundancySolver.cpp)

target_link_libraries(opensim-mrs osimTools mesh)

# Do not generate warnings from OpenSim headers ("SYSTEM").
target_include_directories(opensim-mrs SYSTEM PRIVATE ${OpenSim_INCLUDE_DIRS})