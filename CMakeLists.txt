option(MUSCOLLO_WITH_SNOPT
        "Compile support for the SNOPT optimization library; \
        requires Fortran compiler"
        OFF)
if(MUSCOLLO_WITH_SNOPT)
    project(Muscollo C CXX Fortran) # TODO Fortran only for SNOPT interface.
else()
    project(Muscollo C CXX)
endif()
cmake_minimum_required(VERSION 3.2)

# CMake settings.
# ---------------
# To allow a folder hierarchy within Visual Studio's Solution Explorer.
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Add "_d" to the end of debug libraries, so both debug and non-debug libraries
# can be installed.
set(CMAKE_DEBUG_POSTFIX "_d")

# Avoid need to set DYLD_LIBRARY_PATH on macOS.
set(CMAKE_MACOSX_RPATH ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

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
