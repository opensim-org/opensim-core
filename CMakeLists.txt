cmake_minimum_required(VERSION 3.2)
project(OpenSimMoco VERSION 0.5.0)

# Version.
# --------
set(FULL_VERSION "")
find_package(Git)
if(Git_FOUND)
    # This command provides the annotated tag for the current commit, if it
    # exists, and returns an error if a tag does not exist.
    execute_process(
            COMMAND "${GIT_EXECUTABLE}" describe --exact-match
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            RESULT_VARIABLE GIT_DESCRIBE_EXACT_MATCH_RETVAL
            OUTPUT_VARIABLE GIT_TAG
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
    )
    if(${GIT_DESCRIBE_EXACT_MATCH_RETVAL} EQUAL 0)
        # A tag exists.
        set(FULL_VERSION ${GIT_TAG})
        # Do not add anything to the version.
    else()
        execute_process(
                COMMAND "${GIT_EXECUTABLE}" log -1 --format=%h
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                OUTPUT_VARIABLE GIT_COMMIT_HASH
                OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        # Prepend date for sorting.
        execute_process(
                COMMAND "${GIT_EXECUTABLE}" show -s --format=%cd --date=short HEAD
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                OUTPUT_VARIABLE GIT_COMMIT_DATE
                OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        set(FULL_VERSION
                "${PROJECT_VERSION}-${GIT_COMMIT_DATE}-${GIT_COMMIT_HASH}")
    endif()
else()
    set(FULL_VERSION "${PROJECT_VERSION}?")
endif()
set(MOCO_FULL_VERSION "${FULL_VERSION}" CACHE STRING
        "Full version string, containing git commit date/hash suffix." FORCE)

# Moco settings.
# --------------
# Copy dependencies' libraries into Moco's installation?
set(MOCO_COPY_DEPENDENCIES ON)

option(MOCO_JAVA_BINDINGS "Build Java/MATLAB interface." OFF)

option(MOCO_PYTHON_BINDINGS "Build Python interface." OFF)
set(MOCO_PYTHON_VERSION 3 CACHE STRING
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

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

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
        "${CMAKE_SOURCE_DIR}/../moco_dependencies_install" CACHE PATH
        "The install prefix for the dependencies CMake project.")
get_filename_component(MOCO_DEPENDENCIES_ABSDIR "${MOCO_DEPENDENCIES_DIR}"
        ABSOLUTE)
if(NOT CMAKE_PREFIX_PATH
        AND NOT ADOLC_DIR AND NOT "$ENV{ADOLC_DIR}"
        AND NOT IPOPT_DIR AND NOT "$ENV{IPOPT_DIR}"
        AND EXISTS "${MOCO_DEPENDENCIES_ABSDIR}")
    message(STATUS
            "Attempting to use dependencies from ${MOCO_DEPENDENCIES_ABSDIR}")
    set(dep_install_dirs)
    foreach(dep opensim-core colpack ipopt eigen casadi)
        list(APPEND dep_install_dirs "${MOCO_DEPENDENCIES_ABSDIR}/${dep}")
    endforeach()
    set(CMAKE_PREFIX_PATH "${dep_install_dirs}" CACHE PATH
            "Directories containing dependencies.")
    set(ADOLC_DIR "${MOCO_DEPENDENCIES_ABSDIR}/adol-c" CACHE PATH
            "Path to ADOL-C install directory.")
endif()

# Make targets go in the same binary directory.
# These are CMake-defined variables.
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}")
set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}")

## MOCO_INSTALL_UNIX_FHS option.
option(MOCO_INSTALL_UNIX_FHS
        "Organize installation according to UNIX Filesystem Hierarchy Standard."
        OFF)

if(WIN32)
    # Windows users probably aren't interested in this option.
    mark_as_advanced(MOCO_INSTALL_UNIX_FHS)
endif()

## Set variables describing where everything gets installed.
if(${MOCO_INSTALL_UNIX_FHS})

    # Sets CMAKE_INSTALL_*DIR variables, some of which are used below.
    include(GNUInstallDirs)
    # Do *NOT* try to use the CMAKE_INSTALL_FULL_*DIR variables created by
    # GNUInstallDirs; they are only defined if MOCO_INSTALL_UNIX_FHS is ON.

    # Now set variables that depend on those set by GNUInstallDirs.

    set(MOCO_INSTALL_ARCHIVEDIR "${CMAKE_INSTALL_LIBDIR}")
    set(MOCO_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/OpenSimMoco")
    # Location of the opensim python package in the installation.
    # On Ubuntu/Debian (apt-get), would want lib/python2.7/dist-packages.
    # We replace VERSION with the correct version once we know it (in
    # Bindings/Python/CMakeLists.txt).
    # The _LIBDIR variable might contain something like x86-64-linux-gnu on
    # some systems, but for python, we really just want lib/, no matter what.
    set(MOCO_INSTALL_PYTHONDIR "lib/pythonVERSION/site-packages")
    # share/java, as expected on Ubuntu/Debian (apt-get).
    set(MOCO_INSTALL_JAVAJARDIR "${CMAKE_INSTALL_DATAROOTDIR}/java")
    # Don't want to put source files in share/java.
    set(MOCO_INSTALL_JAVASRCDIR "${CMAKE_INSTALL_DATAROOTDIR}/OpenSim/java")
    set(MOCO_INSTALL_APIEXDIR "${CMAKE_INSTALL_DOCDIR}/Code")
    # Actually determined by OpenSim's OPENSIM_INSTALL_UNIX_FHS:
    set(MOCO_INSTALL_SIMBODYDIR ".")
    set(MOCO_INSTALL_CASADIDIR ".")

else()

    # Use our own installation layout.

    # Set the variables that would have otherwise been set by GNUInstallDirs.
    set(CMAKE_INSTALL_BINDIR bin)
    set(CMAKE_INSTALL_INCLUDEDIR sdk/include)
    set(CMAKE_INSTALL_LIBDIR sdk/lib)
    set(CMAKE_INSTALL_DOCDIR sdk/doc)
    # SYSCONFDIR holds read-only single-machine machine-dependent data.
    set(CMAKE_INSTALL_SYSCONFDIR sdk)

    set(MOCO_INSTALL_ARCHIVEDIR "${CMAKE_INSTALL_LIBDIR}")
    set(MOCO_INSTALL_CMAKEDIR cmake)
    set(MOCO_INSTALL_PYTHONDIR sdk/Python)
    set(MOCO_INSTALL_JAVAJARDIR sdk/Java)
    set(MOCO_INSTALL_JAVASRCDIR sdk/Java)
    set(MOCO_INSTALL_APIEXDIR Resources/Code)
    set(MOCO_INSTALL_SIMBODYDIR sdk/Simbody)
    set(MOCO_INSTALL_CASADIDIR sdk)

endif()

set(MOCO_INSTALL_CPPEXDIR "${MOCO_INSTALL_APIEXDIR}/CPP")
set(MOCO_INSTALL_MATLABEXDIR "${MOCO_INSTALL_APIEXDIR}/Matlab")
set(MOCO_INSTALL_PYTHONEXDIR "${MOCO_INSTALL_APIEXDIR}/Python")

# Cross-platform location of shared libraries. Used in configureOpenSim.m.
if(WIN32)
    set(MOCO_INSTALL_SHAREDLIBDIR "${CMAKE_INSTALL_BINDIR}")
else()
    set(MOCO_INSTALL_SHAREDLIBDIR "${CMAKE_INSTALL_LIBDIR}")
endif()

# This is used to provide the user with information about config. options.
include(FeatureSummary)

# Include CMake macros that we wrote to reduce duplication in this project.
include(cmake/OpenSimMocoMacros.cmake)

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

    if(CMAKE_CXX_COMPILER_ID MATCHES Clang)
        add_compile_options(-Wno-unused-local-typedef)
    endif()
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
# Copy CasADi into our installation.
MocoCopyDLLs(DEP_NAME casadi
        DEP_BIN_DIR "${CASADI_BIN_DIR}" INSTALL_DLLS "${CMAKE_INSTALL_BINDIR}")
if(NOT WIN32)
    # MocoCopyDLLs() only copies DLLs for Windows, but we still need
    # the CasADi shared libraries on other platforms.
    install(DIRECTORY "${CASADI_CMAKE_DIR}/../../../"
            DESTINATION "${CMAKE_INSTALL_PREFIX}/${MOCO_INSTALL_CASADIDIR}"
            USE_SOURCE_PERMISSIONS)
endif()

add_subdirectory(Moco)

add_subdirectory(cmake)

# RunTestsParallel
# ----------------
include(ProcessorCount)
ProcessorCount(PROCESSOR_COUNT)
if(MSVC OR XCODE)
    set(MOCO_TEST_BUILD_CONFIG --build-config ${CMAKE_CFG_INTDIR})
endif()
add_custom_target(RunTestsParallel
        COMMAND "${CMAKE_CTEST_COMMAND}" --parallel ${PROCESSOR_COUNT}
        ${MOCO_TEST_BUILD_CONFIG}
        --output-on-failure)

# Print a list of the dependencies that were found, and the features the user
# chose.
feature_summary(WHAT ALL)

