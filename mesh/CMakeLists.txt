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
# There are Moodule files in the cmake directory.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
# Starting with version 3.3.1, Eigen provides a proper Config file.
find_package(Eigen3 3.3.1 REQUIRED NO_MODULE)

# Avoid using pkg-config, as it is not so easy to get on Windows.
#find_package(PkgConfig REQUIRED)
#
#pkg_check_modules(IPOPT REQUIRED ipopt)
#pkg_check_modules(ADOLC REQUIRED adolc)
#
#link_directories(${IPOPT_LIBRARY_DIRS})
#link_directories(${ADOLC_LIBRARY_DIRS})

find_package(IPOPT REQUIRED)
find_package(Adolc REQUIRED)

set(CMAKE_SHARED_LINKER_FLAGS "${IPOPT_LINK_FLAGS}")


# Subdirectories.
# ---------------
# Build external libraries.
add_subdirectory(external)

add_subdirectory(mesh)

add_subdirectory(tests)


