cmake_minimum_required(VERSION 3.2)
project(Muscollo)

# Muscollo settings.
# ------------------
# Copy dependencies' libraries into Muscollo's installation?
set(MUSCOLLO_COPY_DEPENDENCIES ON)


# CMake settings.
# ---------------
# To allow a folder hierarchy within Visual Studio's Solution Explorer.
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Add "_d" to the end of debug libraries, so both debug and non-debug libraries
# can be installed.
set(CMAKE_DEBUG_POSTFIX "_d")

# Avoid need to set DYLD_LIBRARY_PATH on macOS.
set(CMAKE_MACOSX_RPATH ON)
if(NOT MUSCOLLO_COPY_DEPENDENCIES)
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)
endif()

# Make targets go in the same binary directory.
# These are CMake-defined variables.
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}")
set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}")

# Define variables that help describe the layout of the installation.
include(GNUInstallDirs)
set(MUSCOLLO_INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR})
set(MUSCOLLO_INSTALL_EXAMPLEDIR ${CMAKE_INSTALL_DOCDIR}/Examples)
if(WIN32)
    set(MUSCOLLO_INSTALL_CMAKEDIR cmake)
else()
    set(MUSCOLLO_INSTALL_CMAKEDIR ${CMAKE_INSTALL_LIBDIR}/cmake/Muscollo)
endif()

# This is used to provide the user with information about config. options.
include(FeatureSummary)

# Include CMake macros that we wrote to reduce duplication in this project.
include(cmake/MuscolloMacros.cmake)

# Compiler flags.
# ---------------
set(CMAKE_CXX_STANDARD 11)
# Using c++11 is not optional.
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Weverything")
# TODO treat Ipopt and such as system headers to ignore their warnings.
if(CMAKE_CXX_COMPILER_ID MATCHES Clang # includes AppleClang
        OR CMAKE_CXX_COMPILER_ID STREQUAL GNU)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
    # -Wshorten-64-to-32 helps us catch default warnings from Visual C++.
    if(CMAKE_CXX_COMPILER_ID MATCHES Clang)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wshorten-64-to-32")
    endif()
    # The assert() macro is omitted in Release, causing "unused-variable"
    # warnings. It is sufficient to just catch such warnings in Debug.
    foreach(cfg RELEASE RELWITHDEBINFO MINSIZEREL)
        set(CMAKE_CXX_FLAGS_${cfg}
            "${CMAKE_CXX_FLAGS_${cfg}} -Wno-unused-variable")
        set(CMAKE_CXX_FLAGS_${cfg}
            "${CMAKE_CXX_FLAGS_${cfg}} -Wno-unused-parameter")
    endforeach()
endif()

enable_testing()

add_subdirectory(tropter)

add_subdirectory(Muscollo)

add_subdirectory(cmake)

# Print a list of the dependencies that were found, and the features the user
# chose.
feature_summary(WHAT ALL)

