#
# Build of OpenSim. There are three steps:
#    (1) Choose appropriate platform
#    (2) Locate Simbody and its dependent libraries
#    (3) Build OpenSim libraries and executables
#
#
#----------------------------------------------------

cmake_minimum_required(VERSION 3.15)
project(OpenSim)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
    cmake_policy(SET CMP0005 NEW)
endif(COMMAND cmake_policy)

message(STATUS "NOTICE: Make sure all OpenSim dependencies are built with the \
same CMAKE_BUILD_TYPE (Linux) or CONFIGURATION (Xcode/MSVC) as OpenSim to \
avoid mysterious runtime errors.")

# To allow a folder hierarchy within Visual Studio's Solution Explorer.
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# This is used to provide the user with information about config. options.
include(FeatureSummary)

# OpenSim version.
# ----------------
set(OPENSIM_MAJOR_VERSION 4)
set(OPENSIM_MINOR_VERSION 5)
set(OPENSIM_PATCH_VERSION 1)

# Don't include the patch version if it is 0.
set(PATCH_VERSION_STRING)
if(OPENSIM_PATCH_VERSION)
    set(PATCH_VERSION_STRING ".${OPENSIM_PATCH_VERSION}")
endif()

set(OPENSIM_RELEASE_VERSION
    "${OPENSIM_MAJOR_VERSION}.${OPENSIM_MINOR_VERSION}${PATCH_VERSION_STRING}"
    )

# The OPENSIM_QUALIFIED_VERSION is more granular than OPENSIM_RELEASE_VERSION.
# If git is not found, we simply append "?" to the release version.
# If the checked-out commit is tagged, the qualified version is the tag name.
# If the checked-out commit is not tagged, the qualified version is
#   <release-version>-<git-commit-date>-<git-commit-short-hash>
#   For example: 4.2-2020-07-01-5c10b4176
set(QUALIFIED_VERSION "")
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
        set(QUALIFIED_VERSION ${GIT_TAG})
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
        set(QUALIFIED_VERSION
                "${OPENSIM_RELEASE_VERSION}-${GIT_COMMIT_DATE}-${GIT_COMMIT_HASH}")
    endif()
else()
    set(QUALIFIED_VERSION "${OPENSIM_RELEASE_VERSION}?")
endif()
set(OPENSIM_QUALIFIED_VERSION "${QUALIFIED_VERSION}" CACHE STRING
        "Qualified version string, containing git commit date/hash suffix." FORCE)

# CMake module path.
# ------------------
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(OpenSimMacros)

# Directory in which to install.
# ------------------------------
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    # On Windows, we override the default value because Visual Studio does
    # not, by default, have permission to write to the Program Files
    # directory.
    # On Linux, we override the default value because our installation does
    # not yet conform to the Filesystem Hierarchy Standard (FHS).
    set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}-install" CACHE PATH
        "The directory in which to install this project." FORCE)
endif()


# Make everything go in the same binary directory.
# ------------------------------------------------
# These are CMake-defined variables.
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
link_directories(${CMAKE_BINARY_DIR})


# OpenSim options.
# ----------------

# Specify number of cores to run tests.
include(ProcessorCount)
ProcessorCount(PROCESSOR_COUNT)
set(PROCESSOR_COUNT "${PROCESSOR_COUNT}" CACHE STRING
    "Number of concurrent jobs when running tests via RUN_TESTS_PARALLEL target.")
mark_as_advanced(PROCESSOR_COUNT)

# Try to use MathJax for Doxygen equations.
option(OPENSIM_DOXYGEN_USE_MATHJAX "Try to use MathJax to render Doxygen
equations. If OFF, LATEX is used instead if you have it; otherwise we use
MathJax from the internet (we don't download it). If we cannot download MathJax
from the internet, we turn this option OFF." ON)

# Simbody.
# This will be initialized to the environment variable of the same name
# if it is set, otherwise it will be empty.
set(SIMBODY_HOME $ENV{SIMBODY_HOME} CACHE
    PATH "The location of the Simbody installation to use; you can change this. Set as empty to let CMake search for Simbody automatically.")

option(OPENSIM_COPY_DEPENDENCIES "Copy Simbody and ezc3d into the
OpenSim installation. This should be set to ON when making a relocatable
distribution, and should be set to OFF when packaging for Homebrew, Debian,
etc. On Linux and macOS: we use relative RPATHs when ON, and absolute RPATHs
when OFF. For most users, this should also be on if BUILD_JAVA_WRAPPING or 
BUILD_PYTHON_WRAPPING is ON, so if this OFF, a warning is thrown." ON)
mark_as_advanced(OPENSIM_COPY_DEPENDENCIES)

option(OPENSIM_PYTHON_STANDALONE "Make the Python package standalone, meaning
the OpenSim (and Simbody and ezc3d) shared libraries it depends on are copied
into the package. If you are building OpenSim on the same machine on which you
plan to use it, you can leave this OFF. If you are distributing OpenSim to
other computers, you should turn this ON. If macOS, this option affects how
RPATH is set for the SWIG-generated shared libraries. This variable has no
effect if BUILD_PYTHON_WRAPPING is OFF." OFF)
mark_as_advanced(OPENSIM_PYTHON_STANDALONE) # Mainly for packagers; not users.
# There are many potential ways to distribute the python bindings; here's how
# you might want to set the above option for different ways of distributing:
#   simtk.org GUI distribution, macOS: ON (since no env. vars are set)
#   simtk.org GUI distrubution, Windows: OFF (PATH is set)
#   pypi: ON (there is no way to install the C++ libraries otherwise)
#   debian, homebrew, conda: OFF (can have a separate C++ library package)

option(OPENSIM_PYTHON_CONDA "Build specifically for Conda packaging
which implies NO C++ tests and no runtime as managed by Conda." OFF)
mark_as_advanced(OPENSIM_PYTHON_CONDA)

option(BUILD_API_ONLY "Build/install only headers, libraries,
wrapping, tests; not applications (opensim, ik, rra, etc.)." OFF)

option(OPENSIM_DISABLE_LOG_FILE
"Disable OpenSim from automatically creating 'opensim.log'.

By default, any application linking to osimCommon will create an
'opensim.log' file in the current working directory. All log messages
written by OpenSim (incl. during static initialization) are written to
both this file and the standard output streams." OFF)

option(OPENSIM_DISABLE_STATIC_TYPE_REGISTRATION
"Disable OpenSim from registering bundled `Object` types at static
initialization time.

By default, the static initializer for each OpenSim library will
call `RegisterTypes_osimCommon`. This has the handy side-effect of
making all available `Object` types available at (post-static-init)
runtime, so that (e.g.) downstream code can immediately load XML
files. However, it has the drawback that downstream code cannot
control when, or in which order, registration happens, which can
(e.g.) prevent setting up application-level logging before registering
things.

If this is set to `ON` (i.e. disable static type registration) then
downstream code *must* manually register the OpenSim types it plans
on using by calling `RegisterTypes_osimLIBRARY` (e.g. `RegisterTypes_osimActuators`),
or by manually registering each type (e.g. `Object::registerType(PhysicalOffsetFrame());`)." OFF)
mark_as_advanced(OPENSIM_DISABLE_STATIC_TYPE_REGISTRATION)

if(OPENSIM_DISABLE_LOG_FILE)
    add_definitions(-DOPENSIM_DISABLE_LOG_FILE=1)
endif()
if(OPENSIM_DISABLE_STATIC_TYPE_REGISTRATION)
    add_definitions(-DOPENSIM_DISABLE_STATIC_TYPE_REGISTRATION=1)
endif()

set(OPENSIM_BUILD_INDIVIDUAL_APPS_DEFAULT OFF)
if(WIN32)
    # For backwards compatibility in the Windows binary distribution.
    set(OPENSIM_BUILD_INDIVIDUAL_APPS_DEFAULT ON)
endif()
option(OPENSIM_BUILD_INDIVIDUAL_APPS
    "Build/install the old command line applications (ik, id, rra, cmc, etc.)."
    ${OPENSIM_BUILD_INDIVIDUAL_APPS_DEFAULT})
mark_as_advanced(OPENSIM_BUILD_INDIVIDUAL_APPS)


# Moco settings.
# --------------
option(OPENSIM_WITH_CASADI
        "Build CasADi support for Moco (MocoCasADiSolver)." ON)
add_feature_info(WITH_CASADI OPENSIM_WITH_CASADI
        "Build CasADi support for Moco (OPENSIM_WITH_CASADI)")
option(OPENSIM_WITH_TROPTER
        "Build the tropter optimal control library, for use in MocoTropterSolver." ON)
add_feature_info(WITH_TROPTER OPENSIM_WITH_TROPTER
        "Build the tropter optimal control library (OPENSIM_WITH_TROPTER)")


# Configure installation directories across platforms.
# ----------------------------------------------------

## OPENSIM_INSTALL_UNIX_FHS option.
set(OPENSIM_INSTALL_UNIX_FHS_DEFAULT OFF)
if(UNIX)
    set(OPENSIM_INSTALL_UNIX_FHS_DEFAULT ON)
endif()

option(OPENSIM_INSTALL_UNIX_FHS
    "Organize installation according to UNIX Filesystem Hierarchy Standard."
    ${OPENSIM_INSTALL_UNIX_FHS_DEFAULT})

if(WIN32)
    # Windows users probably aren't interested in this option.
    mark_as_advanced(OPENSIM_INSTALL_UNIX_FHS)
endif()

## Set variables describing where everything gets installed.
if(${OPENSIM_INSTALL_UNIX_FHS})

    # Sets CMAKE_INSTALL_*DIR variables, some of which are used below.
    include(GNUInstallDirs)
    # Do *NOT* try to use the CMAKE_INSTALL_FULL_*DIR variables created by
    # GNUInstallDirs; they are only defined if OPENSIM_INSTALL_UNIX_FHS is ON.

    # Now set variables that depend on those set by GNUInstallDirs.

    # There's a difference between the locations of these files we'd want
    # when using apt-get, and what a build-from-source user would want. For
    # now the focus is making sure that if a build-from-source user installs
    # into /usr or /usr/local, they have a FHS-compliant OpenSim installation.

    set(OPENSIM_INSTALL_ARCHIVEDIR "${CMAKE_INSTALL_LIBDIR}")
    set(OPENSIM_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/OpenSim")
    # Location of the opensim python package in the installation.
    # On Ubuntu/Debian (apt-get), would want lib/python2.7/dist-packages.
    # We replace VERSION with the correct version once we know it (in
    # Bindings/Python/CMakeLists.txt).
    # The _LIBDIR variable might contain something like x86-64-linux-gnu on
    # some systems, but for python, we really just want lib/, no matter what.
    if (OPENSIM_PYTHON_CONDA)
        set(OPENSIM_INSTALL_PYTHONDIR "Lib/site-packages")
    else()
        set(OPENSIM_INSTALL_PYTHONDIR "lib/pythonVERSION/site-packages")
    endif()
    # share/java, as expected on Ubuntu/Debian (apt-get).
    set(OPENSIM_INSTALL_JAVAJARDIR "${CMAKE_INSTALL_DATAROOTDIR}/java")
    # Don't want to put source files in share/java.
    set(OPENSIM_INSTALL_JAVASRCDIR "${CMAKE_INSTALL_DATAROOTDIR}/OpenSim/java")
    set(OPENSIM_INSTALL_APIEXDIR "${CMAKE_INSTALL_DOCDIR}/Code")
    set(OPENSIM_INSTALL_SIMBODYDIR ".")
    set(OPENSIM_INSTALL_SPDLOGDIR ".")
    set(OPENSIM_INSTALL_CASADIDIR ".")

else()

    # Use our own installation layout.

    # Set the variables that would have otherwise been set by GNUInstallDirs.
    set(CMAKE_INSTALL_BINDIR bin)
    set(CMAKE_INSTALL_INCLUDEDIR sdk/include)
    set(CMAKE_INSTALL_LIBDIR sdk/lib)
    set(CMAKE_INSTALL_DOCDIR sdk/doc)
    # SYSCONFDIR holds read-only single-machine machine-dependent data.
    set(CMAKE_INSTALL_SYSCONFDIR sdk)

    set(OPENSIM_INSTALL_ARCHIVEDIR "${CMAKE_INSTALL_LIBDIR}")
    set(OPENSIM_INSTALL_CMAKEDIR cmake)
    set(OPENSIM_INSTALL_PYTHONDIR sdk/Python)
    set(OPENSIM_INSTALL_JAVAJARDIR sdk/Java)
    set(OPENSIM_INSTALL_JAVASRCDIR sdk/Java)
    set(OPENSIM_INSTALL_APIEXDIR Resources/Code)
    set(OPENSIM_INSTALL_SIMBODYDIR sdk/Simbody)
    set(OPENSIM_INSTALL_SPDLOGDIR sdk/spdlog)
    set(OPENSIM_INSTALL_CASADIDIR sdk)

endif()

set(OPENSIM_INSTALL_CPPEXDIR "${OPENSIM_INSTALL_APIEXDIR}/CPP")
set(OPENSIM_INSTALL_MATLABEXDIR "${OPENSIM_INSTALL_APIEXDIR}/Matlab")
set(OPENSIM_INSTALL_PYTHONEXDIR "${OPENSIM_INSTALL_APIEXDIR}/Python")

# Cross-platform location of shared libraries. Used in configureOpenSim.m.
if(WIN32)
    set(OPENSIM_INSTALL_SHAREDLIBDIR "${CMAKE_INSTALL_BINDIR}")
else()
    set(OPENSIM_INSTALL_SHAREDLIBDIR "${CMAKE_INSTALL_LIBDIR}")
endif()


# On UNIX, be careful about installing into system directories.
# -------------------------------------------------------------
get_filename_component(ABS_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}" REALPATH)
if(UNIX AND ("${ABS_INSTALL_PREFIX}" STREQUAL "/usr" OR
             "${ABS_INSTALL_PREFIX}" STREQUAL "/usr/local"))
    set(no_name_conflict ${BUILD_API_ONLY} OR NOT ${OPENSIM_BUILD_INDIVIDUAL_APPS})
    if(${OPENSIM_INSTALL_UNIX_FHS} AND ${no_name_conflict})
        # In this case it's fine to install into /usr or /usr/local.
    else()
        message(SEND_ERROR "
        Cannot install into /usr or /usr/local if the OpenSim installation does
        not conform to the UNIX Filesystem Hierarchy Standard or if building
        the old command-line applications (there are name conflicts with vital
        UNIX executables). Either (a) set OPENSIM_INSTALL_UNIX_FHS to ON, or
        (b) change CMAKE_INSTALL_PREFIX.")
    endif()
endif()


# Platform.
# ---------
# Create a platform name useful for finding things in the Platform
# directory.
if(WIN32)
    set(PLATFORM_NAME Windows)
elseif(APPLE)
    set(PLATFORM_NAME Mac)
elseif(UNIX)
    set(PLATFORM_NAME Linux)
else()
    set(PLATFORM_NAME Unknown)
endif()

# In addition to the platform name we need to know the Application Binary
# Interface (ABI) we're building for. Currently that is either x86, meaning
# 32 bit Intel instruction set, or x64 for 64 bit Intel instruction set.
# Kevin: Since Ubuntu 12 64bit libraries are in lib not lib64 (This in
# response of Sherm's change on Simbody)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(PLATFORM_ABI x64)
else()
    set(PLATFORM_ABI x86)
endif()

set(BUILD_PLATFORM "${PLATFORM_NAME}:${PLATFORM_ABI}" CACHE STRING
    "This is the platform and ABI we're building for. Not changeable here; use a different CMake generator instead."
    FORCE)

if(NOT MSVC AND NOT XCODE AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
        "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
        "MinSizeRel" "RelWithDebInfo")
endif()

## Choose the maximum level of x86 instruction set that the compiler is
## allowed to use. Was using sse2 but changed to let the compilers choose. Most
## will probably use sse2 or later by default.
## On 64 bit MSVC, the default is sse2 and the argument
## isn't recognized so we won't specify it.

if(CMAKE_CL_64)
    set(default_build_inst_set)
else()
    # Here's where we used to set sse2. Leaving this line in
    # case we decide to specify a default value again.
    set(default_build_inst_set)
endif()

## This can be set to a different value by the person running CMake.
set(BUILD_INST_SET ""
    CACHE STRING
    "CPU instruction level compiler is permitted to use (default: let compiler decide).")
mark_as_advanced( BUILD_INST_SET )

if(BUILD_INST_SET)
    set(inst_set_to_use ${BUILD_INST_SET})
else()
    set(inst_set_to_use ${default_build_inst_set})
endif()

## When building in any of the Release modes, tell gcc to use not-quite most
## aggressive optimization and to generate SSE2 floating point instructions.
## Here we are specifying *all* of the Release flags, overriding CMake's
## defaults.  Watch out for optimizer bugs in particular gcc versions!

if(${CMAKE_CXX_COMPILER_ID} MATCHES "GNU" OR
        ${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    if(inst_set_to_use)
        string(TOLOWER ${inst_set_to_use} GCC_INST_SET)
        set(GCC_INST_SET "-m${GCC_INST_SET}")
    else()
        set(GCC_INST_SET)
    endif()

    # Testing with Clang 3.3 on Ubuntu 14.04 shows a 5% decrease
    # in the runtime of the tests when we enable loop unrolling.
    set(GCC_OPT_ENABLE "-funroll-loops")

    # If you know of optimization bugs that affect SimTK in particular
    # gcc versions, this is the place to turn off those optimizations.
    set(GCC_OPT_DISABLE)

    # C++
    set(CMAKE_CXX_FLAGS_DEBUG          "-g ${GCC_INST_SET}"
      CACHE STRING "g++ Debug build compile flags" FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE
      "-DNDEBUG -O2 ${GCC_OPT_ENABLE} ${GCC_OPT_DISABLE} ${GCC_INST_SET}"
      CACHE STRING "g++ Release build compile flags" FORCE)
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO
      "-DNDEBUG -O2 -g ${GCC_OPT_ENABLE} ${GCC_OPT_DISABLE} ${GCC_INST_SET}"
      CACHE STRING "g++ RelWithDebInfo build compile flags" FORCE)
    set(CMAKE_CXX_FLAGS_MINSIZEREL     "-DNDEBUG -Os ${GCC_INST_SET}"
      CACHE STRING "g++ MinSizeRel build compile flags" FORCE)

    # C
    set(CMAKE_C_FLAGS_DEBUG            "-g ${GCC_INST_SET}"
      CACHE STRING "gcc Debug build compile flags" FORCE)
    set(CMAKE_C_FLAGS_RELEASE
      "-DNDEBUG -O2 ${GCC_OPT_ENABLE} ${GCC_OPT_DISABLE} ${GCC_INST_SET}"
      CACHE STRING "gcc Release build compile flags" FORCE)
    set(CMAKE_C_FLAGS_RELWITHDEBINFO
      "-DNDEBUG -O2 -g ${GCC_OPT_ENABLE} ${GCC_OPT_DISABLE} ${GCC_INST_SET}"
      CACHE STRING "gcc RelWithDebInfo build compile flags" FORCE)
    set(CMAKE_C_FLAGS_MINSIZEREL       "-DNDEBUG -Os ${GCC_INST_SET}"
      CACHE STRING "gcc MinSizeRel build compile flags" FORCE)

    add_compile_options(-Wall -Wextra) # -Werror
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

    # Avoid warnings for Object::Self and Object::Super typedefs.
    # This warning exists in GCC and Clang >=4.0.
    if (${CMAKE_CXX_COMPILER_ID} MATCHES "GNU" OR
            NOT (${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS "5.0.0"))
        add_compile_options(-Wno-unused-local-typedefs)
    endif()
endif()

## When building in any of the Release modes, tell VC++ cl compiler to use intrinsics
## (i.e. sqrt instruction rather than sqrt subroutine) with flag /Oi.

if(WIN32 AND ${CMAKE_C_COMPILER} MATCHES "cl")
    if(inst_set_to_use)
        string(TOUPPER ${inst_set_to_use} CL_INST_SET)
        set(CL_INST_SET "/arch:${CL_INST_SET}")
    else()
        set(CL_INST_SET)
    endif()

    set(BUILD_LIMIT_PARALLEL_COMPILES "" CACHE STRING
        "Set a maximum number of simultaneous compilations.")
    mark_as_advanced(BUILD_LIMIT_PARALLEL_COMPILES)
    set(mxcpu ${BUILD_LIMIT_PARALLEL_COMPILES}) # abbreviation

    ## C++
    # Disable "C4068: unknown pragma" warning.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4068") 
    set(CMAKE_CXX_FLAGS_DEBUG
    "/MP${mxcpu} /D _DEBUG /MDd /Od /Ob0 /RTC1 /Zi /bigobj /GS- ${CL_INST_SET}"
        CACHE STRING "VC++ Debug build compile flags" FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE
    "/MP${mxcpu} /D NDEBUG /MD  /O2 /Ob2 /Oi /bigobj /GS- ${CL_INST_SET}"
        CACHE STRING "VC++ Release build compile flags" FORCE)
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO
    "/MP${mxcpu} /D NDEBUG /MD  /O2 /Ob2 /Oi /Zi /bigobj /GS- ${CL_INST_SET}"
        CACHE STRING "VC++ RelWithDebInfo build compile flags" FORCE)
    set(CMAKE_CXX_FLAGS_MINSIZEREL
    "/MP${mxcpu} /D NDEBUG /MD  /O1 /Ob1 /Oi /bigobj /GS- ${CL_INST_SET}"
        CACHE STRING "VC++ MinSizeRel build compile flags" FORCE)

    ## C
    set(CMAKE_C_FLAGS_DEBUG
    "/MP${mxcpu} /D _DEBUG /MDd /Od /Ob0 /RTC1 /Zi /GS- ${CL_INST_SET}"
        CACHE STRING "VC++ Debug build compile flags" FORCE)
    set(CMAKE_C_FLAGS_RELEASE
    "/MP${mxcpu} /D NDEBUG /MD  /O2 /Ob2 /Oi /GS- ${CL_INST_SET}"
        CACHE STRING "VC++ Release build compile flags" FORCE)
    set(CMAKE_C_FLAGS_RELWITHDEBINFO
    "/MP${mxcpu} /D NDEBUG /MD  /O2 /Ob2 /Oi /Zi /GS- ${CL_INST_SET}"
        CACHE STRING "VC++ RelWithDebInfo build compile flags" FORCE)
    set(CMAKE_C_FLAGS_MINSIZEREL
    "/MP${mxcpu} /D NDEBUG /MD  /O1 /Ob1 /Oi /GS- ${CL_INST_SET}"
        CACHE STRING "VC++ MinSizeRel build compile flags" FORCE)

endif()

set(BUILD_JAVA_WRAPPING OFF CACHE BOOL "Build Java wrapping (needed if you're building the GUI or the Matlab wrapping; requires that you have SWIG and Java installed on your machine.)")

set(BUILD_PYTHON_WRAPPING OFF CACHE BOOL "Build Python wrapping (needed if you're building the Python wrapping; requires that you have SWIG and Python installed on your machine.)")

set(OPENSIM_PYTHON_VERSION 3)

if(${BUILD_JAVA_WRAPPING})
    # If we can find a MATLAB installation, we'll try to run some MATLAB tests.
    # To print info on where MATLAB is found: set(MATLAB_FIND_DEBUG ON)
    # Running tests requires that we can find the matlab executable (component
    # MAIN_PROGRAM).
    # Also, the mechanism we use to set the java classpath and library path 
    # for MATLAB requires version 7.0 (R14).
    find_package(Matlab COMPONENTS MAIN_PROGRAM)
    # Issue warning if Matlab and OpenSim build architectures differ.
    # That is, only 64-bit is supported.
    if(Matlab_FOUND)
        if(${Matlab_MEX_EXTENSION} MATCHES ".*64")
            # On Unix, assume host architecture to be same as target 
            # architecture.
            if(UNIX AND NOT ${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES ".*64")
                string(CONCAT MSG 
                       "Matlab found is 64-bit but host is 32-bit. Matlab arch"
                       " and OpenSim binaries arch have to be same.")
                message(WARNING ${MSG})
            elseif(MSVC AND NOT ${CMAKE_GENERATOR_PLATFORM} MATCHES "64")
                string(CONCAT MSG 
                       "Matlab found is 32-bit which is no longer supported.")
                message(WARNING ${MSG})
            endif()
        else()
            string(CONCAT MSG
                       "Matlab 64-bit was not found.")
            message(WARNING ${MSG})
        endif()
    endif()
    # Most users need to copy dependencies if wrapping is on. Warn user
    # if it isn't on.
    if (NOT OPENSIM_COPY_DEPENDENCIES)
        string(CONCAT MSG
           "BUILD_JAVA_WRAPPING is ON but OPENSIM_COPY_DEPENDENCIES is OFF. "
           "Wrapping may not work correctly. Consider switching the "
           "OPENSIM_COPY_DEPENDENCIES flag to ON.")
        message(WARNING ${MSG})
    endif()
endif()

if(${BUILD_PYTHON_WRAPPING})
    set(required_python_version 3)
    set(Python3_ROOT_DIR ""  CACHE PATH "Top level directory containing Python3. For conda environments, this can also be set to the top level directory for a specific environment.")
    find_package(Python3 3.6 REQUIRED COMPONENTS Interpreter Development NumPy)
    message("Python3_FOUND:${Python3_FOUND}")
    message("Python3_VERSION:${Python3_VERSION}")
    message("Python3_Development_FOUND:${Python3_Development_FOUND}")
    message("Python3_LIBRARIES:${Python3_LIBRARIES}")
    message("Python3_Numpy_Include:${Python3_NumPy_INCLUDE_DIRS}")
    message("Python3_Numpy_VERSION:${Python3_NumPy_VERSION}")

    # We update the python install dir to include the python version,
    # now that we know it. We replace the token "VERSION" with the actual python
    # version.
    set(python_version_majmin "${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}")
    string(REPLACE "VERSION" "${python_version_majmin}"
        OPENSIM_INSTALL_PYTHONDIR "${OPENSIM_INSTALL_PYTHONDIR}")
    # This must be done before adding the OpenSim libraries, since
    # OPENSIM_INSTALL_PYTHONDIR is used in OpenSimAddLibrary (in
    # OpenSimMacros.cmake).

    if(APPLE AND ${OPENSIM_PYTHON_VERSION} STREQUAL "2")
        # If you have Homebrew's Python2, then by default, PythonInterp finds
        # Apple's Python, but PythonLibs finds Homebrew's Python, causing
        # runtime crashes. This also occurs if one has Anaconda Python.
        # So we use the python-config executable to get the
        # correct library and include directory.
        # https://github.com/Homebrew/legacy-homebrew/issues/25118
        execute_process(COMMAND "${Python3_EXECUTABLE}-config" --prefix
                    OUTPUT_VARIABLE python_prefix
                    OUTPUT_STRIP_TRAILING_WHITESPACE)
        string(CONCAT pyinc_desc
            "Location of Python header files, to compile bindings. "
            "Must be consistent with PYTHON_EXECUTABLE.")
        set(PYTHON_INCLUDE_DIR
            "${python_prefix}/include/python${python_version_majmin}/"
            CACHE PATH "${pyinc_desc}")

        string(CONCAT pylib_desc
            "Location of Python library, to compile bindings. "
            "Must be consistent with PYTHON_EXECUTABLE.")
        set(PYTHON_LIBRARY
            "${python_prefix}/lib/libpython${python_version_majmin}.dylib"
            CACHE FILEPATH "${pylib_desc}")
    endif()
    
    # Most users need to copy dependencies if wrapping is on. Warn user
    # if it isn't on.
    if (NOT OPENSIM_COPY_DEPENDENCIES)
        string(CONCAT MSG
           "BUILD_PYTHON_WRAPPING is ON but OPENSIM_COPY_DEPENDENCIES is OFF. "
           "Wrapping may not work correctly. Consider switching the "
           "OPENSIM_COPY_DEPENDENCIES flag to ON.")
        message(WARNING ${MSG})
    endif()
endif()

if(WIN32)
    add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
endif()

if (NOT OPENSIM_PYTHON_CONDA)
    include(InstallRequiredSystemLibraries)
endif()

# C++17
# -----
set(CMAKE_CXX_STANDARD 17)
# Using C++17 is not optional.
set(CMAKE_CXX_STANDARD_REQUIRED ON)


## RPATH 
# If it is necessary to not put RPATHs in the installed binaries, set
# CMAKE_SKIP_INSTALL_RPATH to ON.
#set(OPENSIM_USE_INSTALL_RPATH FALSE)
if(UNIX) # Linux and macOS
    # CMake 2.8.12 introduced the ability to set RPATH for shared libraries
    # on OSX. This helps executables find the libraries they depend on
    # without having to set the DYLD_LIBRARY_PATH environment variable. 
    # See http://www.cmake.org/Wiki/CMake_RPATH_handling and `man dydl`.
    # We have attempted to make the RPATH work even if the
    # installation is relocated (using relative paths), but you may need to
    # use DYLD_LIBRARY_PATH (or alter the RPATH using install_name_tool) in
    # these cases.
    ## On Linux, this variable has no effect.
    set(CMAKE_MACOSX_RPATH ON)

    if(NOT OPENSIM_COPY_DEPENDENCIES)
        # Add the automatically determined parts of the RPATH which point
        # to directories outside the build tree to the install RPATH.
        # If we are copying Simbody into OpenSim's installation, then
        # there's no need to link to the libraries in Simbody's or ezc3d's
        # original installations. Furthermore, we may be distributing OpenSim
        # to other computers that will not have our original Simbody or ezc3d
        # installations, and so the RPATH would point to a nonexistant
        # directory on others' computers.
        set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
    endif()

    # Set RPATH so that OpenSim can find its own libraries, but only if
    # OpenSim is not installed into a system library directory.
    list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES
        "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}" isSystemDir)
    if("${isSystemDir}" STREQUAL "-1")
        set(OPENSIM_USE_INSTALL_RPATH TRUE)
    endif()
endif()

set(OPENSIM_DEPENDENCIES_DIR 
    "${CMAKE_SOURCE_DIR}/../opensim_dependencies_install"
    CACHE 
    PATH
    "Directory containing installed binaries of OpenSim dependencies. Set this
     only if you used the Superbuild procedure to install dependencies. ")


# Default paths for finding dependencies.
get_filename_component(OPENSIM_DEPENDENCIES_ABSDIR "${OPENSIM_DEPENDENCIES_DIR}"
        ABSOLUTE)
if(NOT CMAKE_PREFIX_PATH
        AND NOT ADOLC_DIR AND NOT "$ENV{ADOLC_DIR}"
        AND NOT IPOPT_DIR AND NOT "$ENV{IPOPT_DIR}"
        AND EXISTS "${OPENSIM_DEPENDENCIES_ABSDIR}")
    message(STATUS
            "Attempting to use dependencies from ${OPENSIM_DEPENDENCIES_ABSDIR}")
    set(deps_to_find)
    if(OPENSIM_WITH_TROPTER)
        list(APPEND deps_to_find colpack eigen)
    endif()
    if(OPENSIM_WITH_CASADI)
        list(APPEND deps_to_find casadi)
    endif()
    if(OPENSIM_WITH_TROPTER OR OPENSIM_WITH_CASADI)
        list(APPEND deps_to_find ipopt)
    endif()
    set(dep_install_dirs)
    foreach(dep ${deps_to_find})
        list(APPEND dep_install_dirs "${OPENSIM_DEPENDENCIES_ABSDIR}/${dep}")
    endforeach()
    set(CMAKE_PREFIX_PATH "${dep_install_dirs}" CACHE PATH
            "Directories containing dependencies.")
    set(ADOLC_DIR "${OPENSIM_DEPENDENCIES_ABSDIR}/adol-c" CACHE PATH
            "Path to ADOL-C install directory.")
endif()


set(OPENSIM_C3D_PARSER None CACHE STRING
    "Compile OpenSim with a C3D parser ? ezc3d provide C3D reading.")
set_property(CACHE OPENSIM_C3D_PARSER PROPERTY STRINGS "ezc3d" "None")
if(${OPENSIM_C3D_PARSER} STREQUAL "ezc3d")
    set(WITH_EZC3D true)
    set(ezc3d_hint "${OPENSIM_DEPENDENCIES_DIR}/ezc3d/lib/cmake/ezc3d")
    find_package(ezc3d REQUIRED HINTS "${ezc3d_hint}")
    add_definitions(-DWITH_EZC3D)
    set(ezc3d_ROOT_DIR "${ezc3d_DIR}/../../..")
    set(ezc3d_LIBRARY_DIR "${ezc3d_ROOT_DIR}/lib")
    set(ezc3d_INCLUDE_DIR "${ezc3d_ROOT_DIR}/include")
    OpenSimCopyDependencyDLLsForWin(DEP_NAME ezc3d
            DEP_BIN_DIR "${ezc3d_DIR}/../../../bin"
            )
    include_directories("${ezc3d_INCLUDE_DIR}")
    set(ezc3d_LIBRARY "ezc3d")
    if(BUILD_PYTHON_WRAPPING AND OPENSIM_PYTHON_STANDALONE)
        OpenSimInstallDependencyLibraries(ezc3d "${ezc3d_DIR}/../../../bin"
            "${ezc3d_DIR}/../../../lib" "${OPENSIM_INSTALL_PYTHONDIR}/opensim")
    endif()

else()
    set(WITH_EZC3D false)
    unset(ezc3d_LIBRARY_DIR CACHE)
    unset(ezc3d_LIBRARY)
    unset(ezc3d_INCLUDE_DIR CACHE)
endif()

find_package(spdlog REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/spdlog")


if(NOT SIMBODY_HOME AND OPENSIM_DEPENDENCIES_DIR)
    set(SIMBODY_HOME "${OPENSIM_DEPENDENCIES_DIR}/simbody")
endif()


# Find Simbody.
# -------------
# As of Simbody 3.4, Simbody has a SimbodyConfig.cmake file, which is a
# preferred way to find Simbody over the previous FindSimbody.cmake script.
# NO_MODULE means we will not allow the use of a FindSimbody.cmake script.
set(SIMBODY_VERSION_TO_USE 3.7)

# Find Simbody freshly by unsetting this CMake-generated variable.
unset(Simbody_DIR CACHE)
if("${SIMBODY_HOME}" STREQUAL "")
    # We assume the only case in which the user
    # wants us to search for Simbody is if they left SIMBODY_HOME empty.
    # If the user specifies an invalid SIMBODY_HOME by accident,
    # we shouldn't let that fail silently and still search for
    # Simbody elsewhere; they may never realize
    # we are not using their requested installation of Simbody.
    find_package(Simbody ${SIMBODY_VERSION_TO_USE}
        NO_MODULE)
else()
    # Find the package using the user-specified path.
    # NO_DEFAULT_PATH will cause find_package to only
    # look in the provided PATHS.
    find_package(Simbody ${SIMBODY_VERSION_TO_USE}
        PATHS "${SIMBODY_HOME}" NO_MODULE NO_DEFAULT_PATH)
endif()
# This variable appears in the CMake GUI and could confuse users,
# since this variable can't be changed manually.
mark_as_advanced(Simbody_DIR)

# If Simbody is not found, Simbody_FOUND is false.
if(NOT Simbody_FOUND)
    message(FATAL_ERROR "
        Simbody ${SIMBODY_VERSION_TO_USE} not found. Install Simbody and set
        SIMBODY_HOME to the installation directory of Simbody.")
endif()

# Check if Simbody targets were built using same BUILD_TYPE/CONFIGURATION as
# current OpenSim build. If build/configuration types mismatch, stop the build.
# ------------------------------------------------------------------------------
get_property(Simbody_CONFIGURATION
    TARGET   SimTKcommon
    PROPERTY IMPORTED_CONFIGURATIONS)

string(CONCAT MESSAGE
    "Checking if Simbody was built with same BUILD_TYPE or CONFIGURATION as "
    "the current OpenSim BUILD_TYPE or CONFIGURATION.")

if(WIN32)
    string(TOLOWER "${Simbody_CONFIGURATION}" Simbody_CONFIGURATION_STR)
    add_custom_target(Simbody_CONFIG_check ALL
        COMMAND "set" "Simbody_CONFIG=${Simbody_CONFIGURATION_STR}"
        COMMAND "if"
           "\"%Simbody_CONFIG:$<LOWER_CASE:$<CONFIG>>=%\"==\"%Simbody_CONFIG%\""
           "exit" "1"
        COMMENT ${MESSAGE})
endif()

# Copy Simbody DLLs to build tree.
# --------------------------------
# On Windows, we must copy Simbody libraries to the OpenSim build directory
# so that the tests and examples can run without modifying the PATH
# (that is, put Simbody's dll's in the same directory as OpenSim's
# executables and libraries).
OpenSimCopyDependencyDLLsForWin(DEP_NAME Simbody
        DEP_BIN_DIR "${Simbody_BIN_DIR}")
if(WIN32)
    add_dependencies(Copy_Simbody_DLLs Simbody_CONFIG_check)
endif()


# Directories for Simbody headers and libraries for building.
# -----------------------------------------------------------
# TODO no longer necessary; Simbody's library targets provide the 
# include directories.
include_directories("${Simbody_INCLUDE_DIR}")


# CasADi
# ------
if(OPENSIM_WITH_CASADI)
    find_package(casadi 3.4.4 REQUIRED)
    set_package_properties(casadi PROPERTIES
            URL https://web.casadi.org
            TYPE REQUIRED
            PURPOSE "Differentiation and optimizer interface.")
    get_filename_component(CASADI_BIN_DIR "${casadi_DIR}/../" ABSOLUTE)
    # Copy CasADi into our installation.
    OpenSimCopyDependencyDLLsForWin(DEP_NAME casadi
            DEP_BIN_DIR "${CASADI_BIN_DIR}")
    if(OPENSIM_COPY_DEPENDENCIES AND NOT WIN32)
        # MocoCopyDLLs() only copies DLLs for Windows, but we still need
        # the CasADi shared libraries on other platforms.
        install(DIRECTORY "${CASADI_CMAKE_DIR}/../../../"
            DESTINATION "${CMAKE_INSTALL_PREFIX}/${OPENSIM_INSTALL_CASADIDIR}"
                USE_SOURCE_PERMISSIONS)
    endif()
endif()

# IPOPT
# -----

# Avoid using pkg-config on Windows as it's not easy to get
# on Windows. On UNIX, the pkg-config file for Ipopt provides
# a flag that we must use.
if(UNIX)
    find_package(PkgConfig REQUIRED)
endif()

if (OPENSIM_WITH_CASADI OR OPENSIM_WITH_TROPTER)
    if(UNIX)
        pkg_check_modules(IPOPT REQUIRED ipopt IMPORTED_TARGET)
    else()
        set(Ipopt_ROOT_DIR ${OPENSIM_DEPENDENCIES_DIR}/ipopt)
        message("Ipopt_ROOT_DIR ${Ipopt_ROOT_DIR}")
        ##find_package(IPOPT REQUIRED CONFIG)
        # Flags to use when building a shared library.
        # TODO move these flags to a specific target.
        # TODO set(CMAKE_SHARED_LINKER_FLAGS
        # TODO        "${CMAKE_SHARED_LINKER_FLAGS} ${IPOPT_LINK_FLAGS}")
        OpenSimCopyDependencyDLLsForWin(DEP_NAME IPOPT DEP_BIN_DIR "${Ipopt_ROOT_DIR}/bin")
    endif()
endif()

# Installing dependencies into OpenSim's installation.
# ----------------------------------------------------
if(${OPENSIM_COPY_DEPENDENCIES})

    if(WIN32)
        # The issue with install(DIRECTORY) is that it creates directories in
        # the installation for *every* directory in DIRECTORY, even if that
        # dirctory doesn't contain any files matching the provided
        # pattern/regex.
        # We can safely assume that Simbody is installed in an isolated
        # dirctory; therefore, it should be safe to use install(DIRECTORY).
        # This may not be true with conda or chocolatey.
        install(DIRECTORY "${Simbody_ROOT_DIR}/"
                DESTINATION "${OPENSIM_INSTALL_SIMBODYDIR}"
                FILES_MATCHING PATTERN "*SimTK*.dll")
        install(DIRECTORY "${Simbody_ROOT_DIR}/"
                DESTINATION "${OPENSIM_INSTALL_SIMBODYDIR}"
                # There's no need to install Simbody examples or docs.
                PATTERN "*examples/*" EXCLUDE
                PATTERN "*doc/*" EXCLUDE
                # Don't copy lapack, blas, etc. into sdk/Simbody/bin/; they are
                # already copied into bin/.
                PATTERN "*.dll" EXCLUDE
                PATTERN "*.exe" EXCLUDE)
        # DLLs are handled by OpenSimCopyDependencyDLLsForWin(), but we still
        # need to copy simbody-visualizer.exe.
        install(DIRECTORY "${Simbody_BIN_DIR}/"
                DESTINATION "${CMAKE_INSTALL_BINDIR}"
                FILES_MATCHING PATTERN "*.exe")
        # TODO We are copying Simbody DLLs to both the bin and sdk/Simbody/bin
        # folders; we would prefer to only copy to the former, but
        # SimbodyTargets-*.cmake expect dlls in sdk/Simbody/bin. In the future,
        # we can edit the *.cmake files to no longer expect these DLLs.
    else()
        # We would have liked to use install(DIRECTORY) on UNIX, but the
        # dependencies may be installed in system-wide locations and we don't
        # want to copy extraneous files/directories; install(DIRECTORY) creates
        # a directory in DESTINATION for *every* directory in DIRECTORY, even
        # if we do not want to copy any files from some of the directories in
        # DIRECTORY.

        # include
        file(RELATIVE_PATH install_simbodyincludedir
            "${Simbody_ROOT_DIR}" "${Simbody_INCLUDE_DIR}")
        install(DIRECTORY "${Simbody_INCLUDE_DIR}/"
                DESTINATION "${OPENSIM_INSTALL_SIMBODYDIR}/${install_simbodyincludedir}")
    
        # lib (and cmake, pkgconfig)
        file(RELATIVE_PATH install_simbodylibdir 
            "${Simbody_ROOT_DIR}/" "${Simbody_LIB_DIR}/")
        file(GLOB_RECURSE simbody_desired_lib_files
            RELATIVE "${Simbody_LIB_DIR}"
            "${Simbody_LIB_DIR}/*SimTK*" "${Simbody_LIB_DIR}/*Simbody*.cmake"
            "${Simbody_LIB_DIR}/*SampleCMakeLists.txt*")
        foreach(simbody_lib_file ${simbody_desired_lib_files})
            get_filename_component(subdir "${simbody_lib_file}" DIRECTORY)
            install(FILES "${Simbody_LIB_DIR}/${simbody_lib_file}"
                DESTINATION "${OPENSIM_INSTALL_SIMBODYDIR}/${install_simbodylibdir}/${subdir}")
        endforeach()
    
        # simbody-visualizer
        file(RELATIVE_PATH install_simbodyvizdir 
            "${Simbody_ROOT_DIR}" "${Simbody_VIZ_DIR}")
        install(DIRECTORY "${Simbody_VIZ_DIR}/"
                DESTINATION "${OPENSIM_INSTALL_SIMBODYDIR}/${install_simbodyvizdir}"
                USE_SOURCE_PERMISSIONS
                FILES_MATCHING PATTERN "*simbody-visualizer*")
    endif()

    # EZC3D
    # -----
    if(NOT WIN32)
        # LIB: .so files on Linux, .dylib on macOS
        file(GLOB ezc3d_desired_lib_files "${ezc3d_LIBRARY_DIR}/*ezc3d*")
        install(FILES ${ezc3d_desired_lib_files}
                DESTINATION "${CMAKE_INSTALL_LIBDIR}")
    endif()

    # spdlog
    # ------
    install(DIRECTORY "${spdlog_DIR}/../../../"
            DESTINATION "${OPENSIM_INSTALL_SPDLOGDIR}")
endif()

if(BUILD_PYTHON_WRAPPING AND OPENSIM_PYTHON_STANDALONE)
    OpenSimInstallDependencyLibraries(SimTK "${Simbody_BIN_DIR}"
        "${Simbody_LIB_DIR}" "${OPENSIM_INSTALL_PYTHONDIR}/opensim")
endif()


# Other than Windows we can debug without debuggable SimTK libraries
if(WIN32)
    set(CMAKE_DEBUG_POSTFIX "_d" CACHE INTERNAL "" FORCE)
else(WIN32)
    set(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "Suffix for debug libraries")
endif(WIN32)


## The following are required to uses Dart and the Cdash dashboard per Jesse
enable_testing()
include(CTest)

# Sets the number of concurrent jobs that testing can use.
if(MSVC OR XCODE)
    set(OPENSIM_TEST_BUILD_CONFIG --build-config ${CMAKE_CFG_INTDIR})
endif()
add_custom_target(RUN_TESTS_PARALLEL
    COMMAND ${CMAKE_CTEST_COMMAND} --parallel ${PROCESSOR_COUNT}
                                   ${OPENSIM_TEST_BUILD_CONFIG}
                                   --output-on-failure
                                   )

# Create buildinfo.txt file and place under sdk to include product version,
# platform and compiler for troubleshooting purposes
set(VERSION_FILE_PATH ${CMAKE_BINARY_DIR}/OpenSim_buildinfo.txt)
# message("version file="${VERSION_FILE_PATH})
file(WRITE ${VERSION_FILE_PATH} "Product Version=${OPENSIM_MAJOR_VERSION}.${OPENSIM_MINOR_VERSION}")
file(APPEND  ${VERSION_FILE_PATH} "\n")
file(APPEND  ${VERSION_FILE_PATH} "Compiler=${CMAKE_GENERATOR}-${CMAKE_CXX_COMPILER_ID}")
file(APPEND  ${VERSION_FILE_PATH} "\n")
file(APPEND  ${VERSION_FILE_PATH} "Platform=${PLATFORM_NAME}-${PLATFORM_ABI}")
file(APPEND  ${VERSION_FILE_PATH} "\n")
install(FILES ${VERSION_FILE_PATH} DESTINATION "${CMAKE_INSTALL_SYSCONFDIR}")


# Preprocessor definitions.
# -------------------------
# These are used in OpenSim/version.h
set(OPENSIM_SYSTEM_INFO ${CMAKE_SYSTEM})
set(OPENSIM_OS_NAME ${CMAKE_SYSTEM_NAME})

if( WIN32 )
    if (NOT (MSVC_VERSION LESS 2000))
        message(AUTHOR_WARNING "OpenSim::GetCompilerVersion() "
            "in OpenSim/version.h does not handle this newer version "
            "(v${MSVC_VERSION}) of Microsoft Visual C++. "
            "Update OpenSim/version.h and the condition for this warning "
            "in the root CMakeLists.txt.")
    endif()
    set(OPENSIM_COMPILER_INFO ${MSVC_VERSION})
else()
    set(OPENSIM_COMPILER_INFO ${CMAKE_CXX_COMPILER} )
endif()

add_definitions(-DOSIM_SYS_INFO=${OPENSIM_SYSTEM_INFO}
    -DOSIM_COMPILER_INFO=${OPENSIM_COMPILER_INFO}
    -DOSIM_OS_NAME=${OPENSIM_OS_NAME}
    -DOSIM_VERSION=${OPENSIM_QUALIFIED_VERSION})



#-----------------------------------------------------------------------------

# This directory contains files (e.g., .osim) that multiple tests may want to
# use. See the OpenSimCopySharedTestFiles function in
# cmake/OpenSimMacros.cmake.
set(OPENSIM_SHARED_TEST_FILES_DIR "${CMAKE_SOURCE_DIR}/OpenSim/Tests/shared")

add_subdirectory(Vendors)

add_subdirectory(OpenSim)
if(NOT BUILD_API_ONLY)
    add_subdirectory(Applications)
endif()
add_subdirectory(Bindings)

add_subdirectory(cmake)
add_subdirectory(doc)

# Print a list of the dependencies that were found, and the features the user
# chose.
feature_summary(WHAT ALL)
