project(OpenSim)
#
# Build of OpenSim. There are three steps:
#    (1) Choose appropriate platform
#    (2) Locate Simbody and its dependent libraries
#    (3) Build OpenSim libraries and executables
#
#
#----------------------------------------------------

cmake_minimum_required(VERSION 3.2)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
    cmake_policy(SET CMP0005 NEW)
endif(COMMAND cmake_policy)

string(CONCAT DEP_BUILD_MSG
  "\n-------------------------IMPORTANT----------------------------------\n"
  "| Make sure that all OpenSim dependencies are built with the same   |\n"
  "| CMAKE_BUILD_TYPE (Linux) or CONFIGURATION (Xcode/MSVC) as OpenSim |\n"
  "| to avoid mysterious runtime errors.                               |\n"
  "-------------------------IMPORTANT----------------------------------\n")
message(${DEP_BUILD_MSG})

# To create a folder hierarchy within Visual Studio.
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# OpenSim version.
# ----------------
set(OPENSIM_MAJOR_VERSION 4)
set(OPENSIM_MINOR_VERSION 0)
set(OPENSIM_PATCH_VERSION 0)

# Don't include the patch version if it is 0.
set(PATCH_VERSION_STRING)
if(OPENSIM_PATCH_VERSION)
    set(PATCH_VERSION_STRING ".${OPENSIM_PATCH_VERSION}")
endif()

set(OPENSIM_VERSION
    "${OPENSIM_MAJOR_VERSION}.${OPENSIM_MINOR_VERSION}${PATCH_VERSION_STRING}"
    )


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
set(PROCESSOR_COUNT 8 CACHE STRING
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

# TODO rename to OPENSIM_COPY_DEPENDENCIES and handle BTK.
option(OPENSIM_COPY_SIMBODY "Copy Simbody headers and libraries into the
OpenSim installation. This way, you can treat OpenSim and Simbody as one
package, and you only need to set environment variables (e.g., PATH,
LD_LIBRARY_PATH, DYLD_LIBRARY_PATH) for OpenSim.  On Windows, this also copies
Simbody dll's and exe's into the OpenSim build directory, so that you don't
need to set PATH to run OpenSim tests and examples; this is done via the
SimbodyFiles project. If OFF, you likely must set those environment variables
for both OpenSim and Simbody. To make a distributable installation on macOS, 
this should be set to ON to avoid using absolute paths for RPATHs to Simbody
libraries." ON)

option(OPENSIM_PYTHON_STANDALONE "Make the Python package standalone, meaning
the OpenSim (and Simbody and BTK) shared libraries it depends on are copied
into the package. If you are building OpenSim on the same machine on which you
plan to use it, you can leave this OFF. If you are distributing OpenSim to
other computers, you should turn this ON. If macOS, this option affects how
RPATH is set for the SWIG-generated shared libraries. This variable has no
effect if BUILD_PYTHON_WRAPPING is OFF." OFF)
# There are many potential ways to distribute the python bindings; here's how
# you might want to set the above option for different ways of distributing:
#   simtk.org GUI distribution, macOS: ON (since no env. vars are set)
#   simtk.org GUI distrubution, Windows: OFF (PATH is set)
#   pypi: ON (there is no way to install the C++ libraries otherwise)
#   debian, homebrew, conda: OFF (can have a separate C++ library package)

option(BUILD_API_ONLY "Build/install only headers, libraries,
wrapping, tests; not applications (opensim, ik, rra, etc.)." OFF)


set(OPENSIM_BUILD_INDIVIDUAL_APPS_DEFAULT OFF)
if(WIN32)
    # For backwards compatibility in the Windows binary distribution.
    set(OPENSIM_BUILD_INDIVIDUAL_APPS_DEFAULT ON)
endif()
option(OPENSIM_BUILD_INDIVIDUAL_APPS
    "Build/install the old command line applications (ik, id, rra, cmc, etc.)."
    ${OPENSIM_BUILD_INDIVIDUAL_APPS_DEFAULT})
mark_as_advanced(OPENSIM_BUILD_INDIVIDUAL_APPS)


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
    set(OPENSIM_INSTALL_PYTHONDIR "lib/pythonVERSION/site-packages")
    # share/java, as expected on Ubuntu/Debian (apt-get).
    set(OPENSIM_INSTALL_JAVAJARDIR "${CMAKE_INSTALL_DATAROOTDIR}/java")
    # Don't want to put source files in share/java.
    set(OPENSIM_INSTALL_JAVASRCDIR "${CMAKE_INSTALL_DATAROOTDIR}/OpenSim/java")
    set(OPENSIM_INSTALL_CPPEXDIR "${CMAKE_INSTALL_DOCDIR}/APIExamples")
    set(OPENSIM_INSTALL_SCRIPTSDIR "${CMAKE_INSTALL_DOCDIR}/Scripts")

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
    set(OPENSIM_INSTALL_PYTHONDIR sdk/python)
    set(OPENSIM_INSTALL_JAVAJARDIR sdk/Java)
    set(OPENSIM_INSTALL_JAVASRCDIR sdk/Java)
    set(OPENSIM_INSTALL_CPPEXDIR sdk/APIExamples)
    set(OPENSIM_INSTALL_SCRIPTSDIR sdk/Scripts)

endif()

set(OPENSIM_INSTALL_MATLABEXDIR "${OPENSIM_INSTALL_SCRIPTSDIR}/Matlab")
set(OPENSIM_INSTALL_PYTHONEXDIR "${OPENSIM_INSTALL_SCRIPTSDIR}/Python")

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

    # Get the gcc version number in major.minor.build format
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion
                    OUTPUT_VARIABLE GCC_VERSION)


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

if(${BUILD_JAVA_WRAPPING})
    # If we can find a MATLAB installation, we'll try to run some MATLAB tests.
    # To print info on where MATLAB is found: set(MATLAB_FIND_DEBUG ON)
    # Running tests requires that we can find the matlab executable (component
    # MAIN_PROGRAM).
    # Also, the mechanism we use to set the java classpath and library path 
    # for MATLAB requires version 7.0 (R14).
    find_package(Matlab 7.0 COMPONENTS MAIN_PROGRAM)
    # Issue warning if Matlab and OpenSim build architectures differ.
    # That is, one is 64-bit and other is not.
    if(Matlab_FOUND)
        if(${Matlab_MEX_EXTENSION} MATCHES "64")
            # On Unix, assume host architecture to be same as target 
            # architecture.
            if(UNIX AND NOT ${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "64")
                string(CONCAT MSG 
                       "Matlab found is 64-bit but host is 32-bit. Matlab arch"
                       " and OpenSim binaries arch have to be same.")
                message(WARNING ${MSG})
            elseif(MSVC AND NOT ${CMAKE_GENERATOR} MATCHES "64")
                string(CONCAT MSG 
                       "Matlab found is 64-bit but C++ compiler chosen is "
                       "32-bit. Matlab arch and OpenSim binaries arch have"
                       " to be same.")
                message(WARNING ${MSG})
            endif()
        elseif(${Matlab_MEX_EXTENSION} MATCHES "32")
            # On Unix, assume host architecture to be same as target 
            # architecture.
            if(UNIX AND ${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "64")
                string(CONCAT MSG
                       "Matlab found is 32-bit but host is 64-bit. Matlab arch"
                       " and OpenSim binaries arch have to be same.")
                message(WARNING ${MSG})
            elseif(MSVC AND ${CMAKE_GENERATOR} MATCHES "64")
                string(CONCAT MSG
                       "Matlab found is 32-bit but C++ compiler chosen is "
                       "64-bit. Matlab arch and OpenSim binaries arch have"
                       " to be same.")
                message(WARNING ${MSG})
            endif()
        endif()
    endif()
endif()

if(${BUILD_PYTHON_WRAPPING})
    # PythonInterp is supposed to come before PythonLibs.
    find_package(PythonInterp 2.7 REQUIRED)
    find_package(PythonLibs 2.7 REQUIRED)

    # We update the python install dir to include the python version,
    # now that we know it. We replace the token "VERSION" with the actual python
    # version.
    string(REPLACE "VERSION" "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}"
        OPENSIM_INSTALL_PYTHONDIR "${OPENSIM_INSTALL_PYTHONDIR}")
    # This must be done before adding the OpenSim libraries, since
    # OPENSIM_INSTALL_PYTHONDIR is used in OpenSimAddLibrary (in
    # OpenSimMacros.cmake).
endif()

if(WIN32)
    add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
endif()

include(InstallRequiredSystemLibraries)

# C++11
# -----
# In revision 8629, we started using std::unique_ptr, which requires
# C++11 features to be enabled when using GCC or Clang.
if(${CMAKE_C_COMPILER} MATCHES "cc" OR ${CMAKE_C_COMPILER} MATCHES "clang")
    # Using C++11 on OSX requires using libc++ instead of libstd++.
    # libc++ is an implementation of the C++ standard library for OSX.
    if(APPLE)
        if(XCODE)
            set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
            set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
        else()
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
            if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
            endif()
        endif()
    else() # not APPLE
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()
endif()


## RPATH 
# If it is necessary to not put RPATHS in the installed binaries, set
# set CMAKE_SKIP_INSTALL_RPATH to OFF.
if(APPLE)
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

    if(NOT OPENSIM_COPY_SIMBODY)
        # Add the automatically determined parts of the RPATH which point
        # to directories outside the build tree to the install RPATH.
        # If we are copying Simbody into OpenSim's installation, then
        # there's no need to link to the libraries in Simbody's original
        # installation. Furthermore, we may be distributing OpenSim to
        # other computers that will not have our original Simbody
        # installation, and so the RPATH would point to a nonexistant
        # directory on others' computers.
        # TODO handle copying BTK.
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
## Add rpath to the binaries on Linux.
## Note -- The binaries may not be relocatable because the rpath added here is
##         absolute. 
if(PLATFORM_NAME MATCHES Linux)
    set(CMAKE_INSTALL_RPATH
       "${CMAKE_INSTALL_RPATH};${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

set(OPENSIM_DEPENDENCIES_DIR 
    "" 
    CACHE 
    PATH
    "Directory containing installed binaries of OpenSim dependencies. Set this
     only if you used the Superbuild procedure to install dependencies. ")

set(WITH_BTK
    OFF
    CACHE
    BOOL
    "Compile OpenSim with BTK ? BTK provides C3D reading.")

# If compiling with BTK, find and use it.
if(WITH_BTK)
    find_package(BTK 
                 REQUIRED
                 HINTS "${OPENSIM_DEPENDENCIES_DIR}/BTK/share/btk-0.4dev")
    include(${BTK_USE_FILE})
    add_definitions(-DWITH_BTK)
    CopyDependencyDLLsForWin(BTK ${BTK_INSTALL_PREFIX})
    if(BUILD_PYTHON_WRAPPING AND OPENSIM_PYTHON_STANDALONE)
    OpenSimInstallDependencyLibraries(BTK "${BTK_INSTALL_PREFIX}"
        "${BTK_LIBRARY_DIRS}" "${OPENSIM_INSTALL_PYTHONDIR}/opensim")
    endif()
    # TODO this should be handled separately between macOS and Linux.
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${BTK_LIBRARY_DIRS}")
endif()

if(NOT SIMBODY_HOME AND OPENSIM_DEPENDENCIES_DIR)
    set(SIMBODY_HOME "${OPENSIM_DEPENDENCIES_DIR}/simbody")
endif()


# Find Simbody.
# -------------
# As of Simbody 3.4, Simbody has a SimbodyConfig.cmake file, which is a
# preferred way to find Simbody over the previous FindSimbody.cmake script.
# NO_MODULE means we will not allow the use of a FindSimbody.cmake script.
set(SIMBODY_VERSION_TO_USE 3.6)

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

if(${PLATFORM_NAME} STREQUAL Windows)
    string(TOLOWER "${Simbody_CONFIGURATION}" Simbody_CONFIGURATION_STR)
    add_custom_target(Simbody_CONFIG_check ALL
        COMMAND "set" "Simbody_CONFIG=${Simbody_CONFIGURATION_STR}"
        COMMAND "if"
           "\"%Simbody_CONFIG:$<LOWER_CASE:$<CONFIG>>=%\"==\"%Simbody_CONFIG%\""
           "exit" "1"
        COMMENT ${MESSAGE})
elseif(${PLATFORM_NAME} STREQUAL Mac OR ${PLATFORM_NAME} STREQUAL Linux)
    string(TOLOWER "${Simbody_CONFIGURATION}" Simbody_CONFIGURATION_STR)
    string(CONCAT CONTENT "if [[ $1 != *\"$2\"* ]]\; then\n"
                          "exit 1\n"
                          "fi\n")
    file(WRITE "${CMAKE_BINARY_DIR}/simbody_config_check.sh" ${CONTENT})
    add_custom_target(Simbody_CONFIG_check ALL
        COMMAND "bash" "simbody_config_check.sh"
                "\"${Simbody_CONFIGURATION_STR}\"" "\"$<LOWER_CASE:$<CONFIG>>\""
        COMMENT ${MESSAGE})
endif()

# Directories for Simbody headers and libraries for building.
# -----------------------------------------------------------
# TODO no longer necessary; Simbody's library targets provide the 
# include directories.
include_directories("${Simbody_INCLUDE_DIR}")

# Copy files over from the Simbody installation.
# ----------------------------------------------
if(${OPENSIM_COPY_SIMBODY})
    # TODO copy BTK as well.

    # Install Simbody headers into OpenSim installation.
    install(DIRECTORY "${Simbody_INCLUDE_DIR}/"
            DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/Simbody")

    # Figure out which Simbody files need to be installed in the /bin
    # directory and which in the lib directory. Executables go in /bin on all
    # platforms.  Libraries (shared or otherwise) go in /lib, except on Windows
    # where the ".dll" files go in bin along with executables. Anything that
    # is only needed for programmers goes in sdk/lib.
    # TODO use SimbodyConfig.cmake variables instead of these globs.
    if(WIN32)
        file(GLOB SIMBODY_BIN_FILES ${Simbody_BIN_DIR}/*.dll
                                    ${Simbody_BIN_DIR}/*.exe)
        file(GLOB SIMBODY_ARCHIVE_FILES ${Simbody_LIB_DIR}/*.lib)
    else()
        # TODO simbody-visualizer executable probably isn't even there.
        file(GLOB SIMBODY_BIN_FILES
            ${Simbody_BIN_DIR}/simbody*) # executables only (e.g., visualizer).
        # If the LIB_DIR is some common place for libraries (e.g., /usr/lib/),
        # we only want to copy over libraries that have SimTK in their name.
        file(GLOB SIMBODY_LIB_FILES
            ${Simbody_LIB_DIR}/*SimTK*.so
            ${Simbody_LIB_DIR}/*SimTK*.so.*
            ${Simbody_LIB_DIR}/*SimTK*.dylib)
        file(GLOB SIMBODY_ARCHIVE_FILES
            ${Simbody_LIB_DIR}/*SimTK*.a)
    endif()

    # Specify "PROGRAMS" rather than "FILES" so the execute bit gets set.
    install(PROGRAMS    ${SIMBODY_BIN_FILES}
            DESTINATION "${CMAKE_INSTALL_BINDIR}")
    install(FILES       ${SIMBODY_LIB_FILES}
            DESTINATION "${CMAKE_INSTALL_LIBDIR}")
    install(FILES       ${SIMBODY_ARCHIVE_FILES}
            DESTINATION "${OPENSIM_INSTALL_ARCHIVEDIR}")

    # On Windows, we must copy Simbody libraries to the OpenSim build directory
    # so that the tests and examples can run without modifying the PATH
    # (that is, put Simbody's dll's in the same directory as OpenSim's
    # executables and libraries).
    if(WIN32)

        # This is where we're going to put these binaries.
        set(COPIED_LIB_FILES)
        foreach(LIBF ${SIMBODY_BIN_FILES})
            get_filename_component(LIBF_ROOT ${LIBF} NAME)
            set(COPIED_LIB_FILES ${COPIED_LIB_FILES}
                "${CMAKE_BINARY_DIR}/${CMAKE_CFG_INTDIR}/${LIBF_ROOT}")
        endforeach()

        # This target depends on the destination copies of the Simbody files
        # (in the binary build directory).  Those are produced by the OUTPUT
        # custom command below, which depends on the source files from the
        # Simbody installation directory.
        add_custom_target(SimbodyFiles ALL
            DEPENDS Simbody_CONFIG_check ${COPIED_LIB_FILES})
        set_target_properties(SimbodyFiles
            PROPERTIES PROJECT_LABEL "Library - Simbody Files")

        foreach(LIBF ${SIMBODY_BIN_FILES})
            get_filename_component(LIBF_ROOT ${LIBF} NAME)
            get_filename_component(LIBF_SUFFIX ${LIBF} EXT)
            set(COPIED_LIBF "${CMAKE_BINARY_DIR}/${CMAKE_CFG_INTDIR}/${LIBF_ROOT}")
            file(TO_NATIVE_PATH "${LIBF}" LIBF_SRC)
            file(TO_NATIVE_PATH "${COPIED_LIBF}" LIBF_DEST)

            # Copy Simbody files if they are out of date. This is invoked
            # because the SimbodyFiles target depends on these output files.
            add_custom_command(OUTPUT "${COPIED_LIBF}"
                COMMAND ${CMAKE_COMMAND} -E copy "${LIBF_SRC}" "${LIBF_DEST}"
                DEPENDS "${LIBF}"
                COMMENT "Copy ${LIBF_SRC} -> ${LIBF_DEST}"
                VERBATIM)
        endforeach()
    endif()
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
    if (${MSVC_VERSION} GREATER 1910)
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
    -DOSIM_VERSION=${OPENSIM_VERSION})



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
