cmake_minimum_required(VERSION 3.2)
project(Moco VERSION 0.1)

# Moco settings.
# ------------------
# Copy dependencies' libraries into Moco's installation?
set(MOCO_COPY_DEPENDENCIES ON)

option(MOCO_JAVA_BINDINGS "Build Java/MATLAB interface." OFF)

option(MOCO_PYTHON_BINDINGS "Build Python interface." OFF)
set(MOCO_PYTHON_VERSION 2 CACHE STRING
    "The major Python version (2 or 3) for which to build the wrapping.")
# To create a drop-down in the CMake GUI:
set_property(CACHE MOCO_PYTHON_VERSION PROPERTY STRINGS "2" "3")

option(MOCO_BUILD_EXECUTABLE
    "Build the opensim-moco executable." ON)
option(MOCO_BUILD_EXAMPLES
    "Build, test, and install Moco C++ examples." ON)

option(MOCO_WITH_TROPTER
    "Build the tropter optimal control library." ON)
if(NOT MOCO_WITH_TROPTER)
    # TODO
    message(WARNING "MOCO_WITH_TROPTER=OFF is not supported yet.")
endif()


# CMake settings.
# ---------------
# To allow a folder hierarchy within Visual Studio's Solution Explorer.
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Add "_d" to the end of debug libraries, so both debug and non-debug libraries
# can be installed.
set(CMAKE_DEBUG_POSTFIX "_d")

# Avoid need to set DYLD_LIBRARY_PATH on macOS.
set(CMAKE_MACOSX_RPATH ON)
if(NOT MOCO_COPY_DEPENDENCIES)
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)
endif()

# Directory in which to install.
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}-install" CACHE PATH
        "The directory in which to install this project." FORCE)
endif()

# Set the default for CMAKE_BUILD_TYPE.
# CMAKE_BUILD_TYPE is only applicable for single-configuration generators.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
        "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

# Default paths for finding dependencies.
set(MOCO_DEPENDENCIES_DIR
        "${CMAKE_SOURCE_DIR}/../moco_dependencies_install")
get_filename_component(MOCO_DEPENDENCIES_DIR "${MOCO_DEPENDENCIES_DIR}"
        ABSOLUTE)
if(NOT CMAKE_PREFIX_PATH
        AND NOT ADOLC_DIR AND NOT "$ENV{ADOLC_DIR}"
        AND NOT IPOPT_DIR AND NOT "$ENV{IPOPT_DIR}"
        AND EXISTS "${MOCO_DEPENDENCIES_DIR}")
    message(STATUS
            "Attempting to use dependencies from ${MOCO_DEPENDENCIES_DIR}")
    set(dep_install_dirs)
    foreach(dep opensim-core colpack ipopt eigen casadi)
        list(APPEND dep_install_dirs "${MOCO_DEPENDENCIES_DIR}/${dep}")
    endforeach()
    set(CMAKE_PREFIX_PATH "${dep_install_dirs}" CACHE PATH
            "Directories containing dependencies.")
    set(ADOLC_DIR "${MOCO_DEPENDENCIES_DIR}/adol-c" CACHE PATH
            "Path to ADOL-C install directory.")
endif()

# Make targets go in the same binary directory.
# These are CMake-defined variables.
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}")
set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}")

# Define variables that help describe the layout of the installation.
include(GNUInstallDirs)
set(MOCO_INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR})
set(MOCO_INSTALL_EXAMPLEDIR ${CMAKE_INSTALL_DOCDIR}/Examples)
if(WIN32)
    set(MOCO_INSTALL_CMAKEDIR cmake)
    set(MOCO_INSTALL_PYTHONDIR sdk/python)
    set(MOCO_INSTALL_SCRIPTSDIR sdk/Scripts)
    set(CMAKE_INSTALL_SYSCONFDIR sdk)
    set(MOCO_INSTALL_SHAREDLIBDIR "${CMAKE_INSTALL_BINDIR}")
else()
    set(MOCO_INSTALL_CMAKEDIR ${CMAKE_INSTALL_LIBDIR}/cmake/Moco)
    set(MOCO_INSTALL_PYTHONDIR "lib/pythonVERSION/site-packages")
    set(MOCO_INSTALL_SCRIPTSDIR "${CMAKE_INSTALL_DOCDIR}/Scripts")
    set(MOCO_INSTALL_SHAREDLIBDIR "${CMAKE_INSTALL_LIBDIR}")
endif()
set(MOCO_INSTALL_JAVAJARDIR "${CMAKE_INSTALL_DATAROOTDIR}/java")
set(MOCO_INSTALL_MATLABEXDIR "${MOCO_INSTALL_SCRIPTSDIR}/Matlab")
set(MOCO_INSTALL_PYTHONEXDIR "${MOCO_INSTALL_SCRIPTSDIR}/Python")

# This is used to provide the user with information about config. options.
include(FeatureSummary)

# Include CMake macros that we wrote to reduce duplication in this project.
include(cmake/MocoMacros.cmake)

# Compiler flags.
# ---------------
set(CMAKE_CXX_STANDARD 11)
# Using c++11 is not optional.
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Weverything")
if(CMAKE_CXX_COMPILER_ID MATCHES Clang # includes AppleClang
        OR CMAKE_CXX_COMPILER_ID STREQUAL GNU)
    add_compile_options(-Wall -Wextra -Werror)
    # -Wshorten-64-to-32 helps us catch default warnings from Visual C++.
    if(CMAKE_CXX_COMPILER_ID MATCHES Clang)
        add_compile_options(-Wshorten-64-to-32)
        # TODO Find clang/gcc equivalent to C4267; -Wconversion gives way too
        # many warnings.
    endif()
    # The assert() macro is omitted in Release, causing "unused-variable"
    # warnings. It is sufficient to just catch such warnings in Debug.
    add_compile_options($<$<NOT:$<CONFIG:Debug>>:-Wno-unused-variable>
                        $<$<NOT:$<CONFIG:Debug>>:-Wno-unused-parameter>)

    add_compile_options(-Wno-unused-local-typedef)
endif()

# TODO add /WX flag for Windows.

enable_testing()
include(CTest)

if(MOCO_WITH_TROPTER)
    add_subdirectory(tropter)
endif()

find_package(casadi 3.4.4 REQUIRED)
set_package_properties(casadi PROPERTIES
        URL https://web.casadi.org
        TYPE REQUIRED
        PURPOSE "Differentiation and optimizer interface.")
get_filename_component(CASADI_BIN_DIR "${casadi_DIR}/../" ABSOLUTE)
MocoCopyDLLs(DEP_NAME casadi
        DEP_BIN_DIR "${CASADI_BIN_DIR}" INSTALL_DLLS "${CMAKE_INSTALL_BINDIR}")

add_subdirectory(Moco)

add_subdirectory(cmake)

# Print a list of the dependencies that were found, and the features the user
# chose.
feature_summary(WHAT ALL)

