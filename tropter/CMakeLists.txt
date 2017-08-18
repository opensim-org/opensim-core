project(tropter) # TODO another name: Trop (trajectory optimization)
# TODO TROP
cmake_minimum_required(VERSION 3.2)

# CMake settings.
# ---------------
# To allow a folder hierarchy within Visual Studio's Solution Explorer.
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Compiler flags.
# ---------------
set(CMAKE_CXX_STANDARD 11)
# Using c++11 is not optional.
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include CMake macros that we wrote to reduce duplication in this project.
include(cmake/tropter_macros.cmake)

# Organization of installation.
# -----------------------------
set(TROPTER_INSTALL_INCLUDEDIR include/tropter)

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

find_package(ADOLC REQUIRED)

# OpenMP allows parallelization (of evaluating dynaimcs and path constraints
# across tropter points) and is usually a feature of a compiler. It's fine if the
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
# However, Homebrew's ADOL-C and ColPack seem to be compiled with Homebrew's
# GCC. If using ADOL-C and ColPack from Hoembrew, then it is best to build
# this project using Homebrew's GCC as well:
#    $ cmake <source-dir> -DCMAKE_C_COMPILER=/usr/local/bin/gcc-6 \
#                         -DCMAKE_CXX_COMPILER=/usr/local/bin/g++-6
find_package(OpenMP)


# Subdirectories.
# ---------------
# Build external libraries.
add_subdirectory(external)

add_subdirectory(tropter)

add_subdirectory(tests)


