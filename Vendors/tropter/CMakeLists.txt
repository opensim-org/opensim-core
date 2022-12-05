# WINDOWS_EXPORT_ALL_SYMBOLS requires 3.4.
# PkgConfig import targets requires 3.6.
cmake_minimum_required(VERSION 3.6)
project(tropter)

# Include CMake macros that we wrote to reduce duplication in this project.
include(cmake/tropter_macros.cmake)

# CMake settings.
# ---------------
# To allow a folder hierarchy within Visual Studio's Solution Explorer.
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Compiler flags.
# ---------------
set(CMAKE_CXX_STANDARD 11)
# Using c++11 is not optional.
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Copy dependencies' libraries into tropter's installation?
set(TROPTER_COPY_DEPENDENCIES ON)

# Configurable settings.
# ----------------------
option(TROPTER_WITH_OPENMP "Parallelize derivative calculations with OpenMP."
        OFF)
add_feature_info(OpenMP TROPTER_WITH_OPENMP
        "Parallelize derivative calculations (TROPTER_WITH_OPENMP)")

option(TROPTER_WITH_SNOPT
        "Include support for the SNOPT optimization library; \
        precompiled C++ interface libraries for SNOPT required."
        OFF)
add_feature_info(WITH_SNOPT TROPTER_WITH_SNOPT
    "Support the SNOPT optimization library (TROPTER_WITH_SNOPT)")
if(TROPTER_WITH_SNOPT AND NOT MSVC)
    # On MSVC, we do not try to compile Fortran sources.
    enable_language(Fortran)
endif()


# Organization of installation.
# -----------------------------
include(GNUInstallDirs)
set(TROPTER_INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR}/tropter)
if(WIN32)
    set(TROPTER_INSTALL_CMAKEDIR cmake)
else()
    set(TROPTER_INSTALL_CMAKEDIR ${CMAKE_INSTALL_LIBDIR}/cmake/tropter)
endif()

# Find dependencies.
# ------------------
# There are Moodule files in the cmake directory.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# TODO find_package(FMT 4.0 REQUIRED)

# Starting with version 3.3.1, Eigen provides a proper Config file.
find_package(Eigen3 3.3.1 REQUIRED NO_MODULE)
set_package_properties(Eigen3 PROPERTIES
        URL https://eigen.tuxfamily.org
        TYPE REQUIRED
        PURPOSE "Matrix library")

# ColPack can now use CMake to be built.
find_package(ColPack 1.0.10 REQUIRED HINTS "${OPENSIM_DEPENDENCIES_DIR}/colpack")
set_package_properties(ColPack PROPERTIES
        URL https://github.com/CSCsw/ColPack
        TYPE REQUIRED
        PURPOSE "Exploiting sparsity structure of Jacobian")

find_package(ADOLC REQUIRED)
set_package_properties(ADOLC PROPERTIES
        URL https://projects.coin-or.org/ADOL-C
        TYPE REQUIRED
        PURPOSE "Computing derivatives for optimization")
tropter_copy_dlls(DEP_NAME ADOLC DEP_INSTALL_DIR "${ADOLC_DIR}/bin")

# OpenMP allows parallelization (of evaluating dynamics and path constraints
# across tropter points) and is usually a feature of a compiler. It's fine if
# the compiler does not support OpenMP.
# On macOS Sierra 10.12, AppleClang does not support OpenMP. You can install
# LLVM 3.9 with Clang 3.9, which does support OpenMP:
#    $ brew install llvm
#    $ xcode-select --install # if the compiler can't find wchar.h
# Then run CMake with the following flags:
#    $ cmake <source-dir> -DCMAKE_C_COMPILER=/usr/local/opt/llvm/bin/clang \
#                         -DCMAKE_CXX_COMPILER=/usr/local/opt/llvm/bin/clang++ \
#                         -DCMAKE_EXE_LINKER_FLAGS="-L/usr/local/opt/llvm/lib"
# However, Homebrew's ADOL-C and ColPack seem to be compiled with Homebrew's
# GCC. If using ADOL-C and ColPack from Homebrew, then it is best to build
# this project using Homebrew's GCC as well:
#    $ cmake <source-dir> -DCMAKE_C_COMPILER=/usr/local/bin/gcc-6 \
#                         -DCMAKE_CXX_COMPILER=/usr/local/bin/g++-6
# Note that you can force Homebrew to build these libraries using Clang as
# follows:
#    $ brew install --cc=clang colpack
#    $ brew install --cc=clang adol-c
if(TROPTER_WITH_OPENMP)
    message(WARNING "We don't actually make use of OpenMP yet.")
    find_package(OpenMP REQUIRED)
endif()


# Subdirectories.
# ---------------
add_subdirectory(doc)

# Build external libraries.
add_subdirectory(external)

add_subdirectory(tropter)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

add_subdirectory(sandbox)

# Allowing use of tropter in other libraries.
# -------------------------------------------
# export(TARGETS tropter FILE tropter-config.cmake)

# TODO rename to tropter-targets and create proper tropter-config.cmake
# and tropter-config-version.cmake files.
install(EXPORT tropter-config DESTINATION ${TROPTER_INSTALL_CMAKEDIR})

add_subdirectory(cmake)


