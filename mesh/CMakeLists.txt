project(mesh) # TODO another name: Trop (trajectory optimization)
# TODO TROP
cmake_minimum_required(VERSION 3.2)

set(CMAKE_CXX_STANDARD 11)
# Using c++11 is not optional.
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
# Flags to use when building a shared library.
# TODO move these flags to a specific target.
set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} ${IPOPT_LINK_FLAGS}")

find_package(Adolc REQUIRED)

# OpenMP allows parallelization (of evaluating dynaimcs and path constraints
# across mesh points) and is usually a feature of a compiler. It's fine if the
# compiler does not support OpenMP.
# On macOS Sierra 10.12, AppleClang does not support OpenMP. You can install
# LLVM 3.9 with Clang 3.9, which does support OpenMP:
#    $ brew install llvm
#    $ xcode-select --install # if the compiler can't find wchar.h
# Then run CMake with the following flags:
#    $ cmake <source-dir> -DCMAKE_C_COMPILER=/usr/local/opt/llvm/bin/clang \
#                         -DCMAKE_CXX_COMPILER=/usr/local/opt/llvm/bin/clang++ \
#                         -DCMAKE_EXE_LINKER_FLAGS="-L/usr/local/opt/llvm/lib"
#                         -DCMAKE_CXX_FLAGS="-std=c++11"
# It seems like CMake doesn't know how to specify the C++11 standard flag for
# LLVM Clang (non-AppleClang), so we do it manually.
#   TODO might want to modify other linker flags (e.g., SHARED, STATIC)?
find_package(OpenMP)


# Subdirectories.
# ---------------
# Build external libraries.
add_subdirectory(external)

add_subdirectory(mesh)

add_subdirectory(tests)


