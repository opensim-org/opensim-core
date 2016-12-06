project(mesh) # TODO another name: Trop (trajectory optimization)
# TODO TROP
cmake_minimum_required(VERSION 3.2)

set(CMAKE_CXX_STANDARD 11)

# Include CMake macros that we wrote to reduce duplication in this project.
include(cmake/mesh_macros.cmake)

# Organization of installation.
# -----------------------------
set(MESH_INSTALL_INCLUDEDIR include/mesh)

# Find dependencies.
# ------------------
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Eigen3 REQUIRED)
# Starting with version 3.3.1, Eigen provides a proper Config file.
# TODO find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)

pkg_check_modules(IPOPT REQUIRED ipopt)
pkg_check_modules(ADOLC REQUIRED adolc)

link_directories(${IPOPT_LIBRARY_DIRS})
link_directories(${ADOLC_LIBRARY_DIRS})

add_subdirectory(mesh)
# TODO add_executable(mesh mesh.h mesh.cpp)

add_subdirectory(tests)


