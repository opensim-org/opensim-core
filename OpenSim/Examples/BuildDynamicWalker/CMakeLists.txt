# This is a CMake file for building a project that uses OpenSim.
# This is the minimum version of CMake that OpenSim requires:
cmake_minimum_required(VERSION 3.2)
project(BuildDynamicWalker)

# OpenSim requires a compiler that supports C++20.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the OpenSim libraries and header files.
# This command searches for the file OpenSimConfig.cmake
# in common system directories and in OPENSIM_INSTALL_DIR.
find_package(OpenSim 4.0 REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

# By not listing "Debug", we prevent Visual Studio from using
# Debug by default, which does not work with the OpenSim API
# distributed via the GUI.
set(CMAKE_CONFIGURATION_TYPES "RelWithDebInfo;Release"
        CACHE STRING "Supported configuration types." FORCE)

# Define an executable to be created from the provided C++ source files.
add_executable(BuildDynamicWalker BuildDynamicWalkerModel.cpp)

# Link to OpenSim and Simbody libraries.
target_link_libraries(BuildDynamicWalker osimTools)


