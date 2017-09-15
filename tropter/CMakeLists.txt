cmake_minimum_required(VERSION 3.2)
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

# Copy dependencies' libraries into Tropter's installation?
set(TROPTER_COPY_DEPENDENCIES ON)

# Configurable settings.
# ----------------------
option(TROPTER_WITH_SNOPT
        "Compile support for the SNOPT optimization library; \
        requires Fortran compiler"
        OFF)
if(TROPTER_WITH_SNOPT)
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

find_package(FMT 4.0 REQUIRED)

# Starting with version 3.3.1, Eigen provides a proper Config file.
find_package(Eigen3 3.3.1 REQUIRED NO_MODULE)

# Avoid using pkg-config on Windows as it's not easy to get
# on Windows. On Ubuntu, the pkg-config file for Ipopt provides
# a flag that we must use.
if(UNIX AND NOT APPLE)
    set(LINUX TRUE)
endif()
if(LINUX)
    find_package(PkgConfig REQUIRED)
endif()

# ColPack can now use CMake to be built.
find_package(ColPack 1.0.10 REQUIRED)
tropter_copy_dlls(ColPack "${ColPack_ROOT_DIR}/bin")

find_package(ADOLC REQUIRED)
tropter_copy_dlls(ADOLC "${ADOLC_DIR}/bin")

if(LINUX)
    pkg_check_modules(IPOPT REQUIRED ipopt)
    link_directories(${IPOPT_LIBRARY_DIRS})
else()
    find_package(IPOPT REQUIRED)
    # Flags to use when building a shared library.
    # TODO move these flags to a specific target.
    set(CMAKE_SHARED_LINKER_FLAGS
            "${CMAKE_SHARED_LINKER_FLAGS} ${IPOPT_LINK_FLAGS}")
    tropter_copy_dlls(IPOPT "${IPOPT_DIR}/bin")
endif()

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
find_package(OpenMP)


# Subdirectories.
# ---------------
add_subdirectory(doc)

# Build external libraries.
add_subdirectory(external)

add_subdirectory(tropter)

add_subdirectory(tests)

add_subdirectory(sandbox)


# Allowing use of tropter in other libraries.
# -------------------------------------------
# export(TARGETS tropter FILE tropter-config.cmake)

install(EXPORT tropter-config DESTINATION ${TROPTER_INSTALL_CMAKEDIR})