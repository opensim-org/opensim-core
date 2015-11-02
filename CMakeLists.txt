project(OpenSim)
#
# Build of OpenSim. There are three steps:
#    (1) Choose appropriate platform
#    (2) Locate Simbody and its dependent libraries
#    (3) Build OpenSim libraries and executables
#
#
#----------------------------------------------------

cmake_minimum_required(VERSION 2.8.8)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
    cmake_policy(SET CMP0005 NEW)
endif(COMMAND cmake_policy)

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


# Make everything go in the same binary directory. (These are CMake-defined
# variables.)
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

option(OPENSIM_COPY_SIMBODY "Copy Simbody headers and libraries into the
OpenSim installation. This way, you can treat OpenSim and Simbody as one
package, and you only need to set environment variables (e.g., PATH,
LD_LIBRARY_PATH, DYLD_LIBRARY_PATH) for OpenSim.  On Windows, this also copies
Simbody dll's and exe's into the OpenSim build directory, so that you don't
need to set PATH to run OpenSim tests and examples; this is done via the
SimbodyFiles project. If OFF, you likely must set those environment variables
for both OpenSim and Simbody." ON)

option(BUILD_API_ONLY "Build/install only headers, libraries,
wrapping, tests; not applications (ik, rra, etc.)." OFF)


# Don't install applications into system directories on UNIX.
get_filename_component(ABS_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}" REALPATH)
if(UNIX AND 
        ("${ABS_INSTALL_PREFIX}" STREQUAL "/usr" OR
        "${ABS_INSTALL_PREFIX}" STREQUAL "/usr/local") AND
        NOT ${BUILD_API_ONLY})
    message(SEND_ERROR "Cannot install into /usr or /usr/local if building
    applications, since OpenSim executables conflict with vital UNIX
    executables. Either set BUILD_API_ONLY to ON or change
    CMAKE_INSTALL_PREFIX.")
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
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Debug or Release build" FORCE)
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

set(BUILD_JAVA_WRAPPING OFF CACHE BOOL "Build Java wrapping (needed if you're building the GUI and have SWIG and Java installed on your machine.)")

set(BUILD_PYTHON_WRAPPING OFF CACHE BOOL "Build Python wrapping (needed if you're building the Python wrapping and have SWIG and Python installed on your machine.)")

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


## On APPLE, use MACOSX_RPATH.
set(OPENSIM_USE_INSTALL_RPATH FALSE)
if(APPLE AND NOT (${CMAKE_VERSION} VERSION_LESS 2.8.12))
    option(OPENSIM_NO_LIBRARY_PATH_ENV_VAR
        "If ON, you don't need to set the LD_LIBRARY_PATH (Linux) or
        DYLD_LIBRARY_PATH (Mac) to make use of OpenSim's executables.
        Instead, we bake the location of OpenSim's libraries into the
        executables, so the executables already know where to find the
        libraries. This uses the RPATH mechanism." ON)
    if(${OPENSIM_NO_LIBRARY_PATH_ENV_VAR})
        # CMake 2.8.12 introduced the ability to set RPATH for shared libraries on
        # OSX. This helps executables find the libraries they depend on without
        # having to set the DYLD_LIBRARY_PATH environment variable.
        # The code below is all taken from the link below, and implements the
        # scenario "Always use full RPATH".
        # http://www.cmake.org/Wiki/CMake_RPATH_handling
        # Note: the RPATH won't succeed if the libraries are moved; in this case,
        # one will still need to set DYLD_LIBRARY_PATH (or alter the RPATH in the
        # executables/libraries that depend on Simbody's libraries).
        set(CMAKE_MACOSX_RPATH ON)

        set(CMAKE_INSTALL_FULL_LIBDIR "${CMAKE_INSTALL_PREFIX}/lib")

        # Add the automatically determined parts of the RPATH
        # which point to directories outside the build tree to the install RPATH.
        set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

        # The RPATH to be used when installing, but only if it's not a system
        # directory.
        list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES
            "${CMAKE_INSTALL_FULL_LIBDIR}" isSystemDir)
        if("${isSystemDir}" STREQUAL "-1")
            set(OPENSIM_USE_INSTALL_RPATH TRUE)
        endif()
    endif()
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

# Directories for Simbody headers and libraries for building.
# -----------------------------------------------------------
include_directories("${Simbody_INCLUDE_DIR}")
link_directories("${Simbody_LIB_DIR}")


# Copy files over from the Simbody installation.
# ----------------------------------------------
if(${OPENSIM_COPY_SIMBODY})

    # Install Simbody headers into OpenSim installation.
    install(DIRECTORY "${Simbody_INCLUDE_DIR}" DESTINATION sdk/include/SimTK)

    # Figure out which Simbody files need to be installed in the /bin
    # directory and which in the lib directory. Executables go in /bin on all
    # platforms.  Libraries (shared or otherwise) go in /lib, except on Windows
    # where the ".dll" files go in bin along with executables. Anything that
    # is only needed for programmers goes in sdk/lib.
    if(WIN32)
        file(GLOB SIMBODY_BIN_FILES
            ${Simbody_BIN_DIR}/*.dll
            ${Simbody_BIN_DIR}/*.exe)
        file(GLOB SIMBODY_SDKLIB_FILES
            ${Simbody_LIB_DIR}/*.lib)
    else()
        file(GLOB SIMBODY_BIN_FILES
            ${Simbody_BIN_DIR}/simbody*) # executables only (e.g., visualizer).
        # If the LIB_DIR is some common place for libraries (e.g., /usr/lib/),
        # we only want to copy over libraries that have SimTK in their name.
        file(GLOB SIMBODY_LIB_FILES
            ${Simbody_LIB_DIR}/*SimTK*.so
            ${Simbody_LIB_DIR}/*SimTK*.so.*
            ${Simbody_LIB_DIR}/*SimTK*.dylib)
        file(GLOB SIMBODY_SDKLIB_FILES
            ${Simbody_LIB_DIR}/*SimTK*.a)
    endif()

    # Specify "PROGRAMS" rather than "FILES" so the execute bit gets set.
    install(PROGRAMS ${SIMBODY_BIN_FILES} DESTINATION bin)
    install(FILES ${SIMBODY_LIB_FILES}    DESTINATION lib)
    install(FILES ${SIMBODY_SDKLIB_FILES} DESTINATION sdk/lib)

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
        add_custom_target(SimbodyFiles ALL DEPENDS ${COPIED_LIB_FILES})
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


# Where do build products go in the build tree?
# ---------------------------------------------
if(NOT LIBRARY_OUTPUT_PATH)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR} CACHE INTERNAL
        "Single output directory for building all libraries.")
endif()
if(NOT EXECUTABLE_OUTPUT_PATH)
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR} CACHE INTERNAL
      "Single output directory for building all executables.")
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


# Create buildinfo.txt file and place under sdk to include product version, platform and compiler for troubleshooting purposes
set(VERSION_FILE_PATH ${CMAKE_BINARY_DIR}/buildinfo.txt)
# message("version file="${VERSION_FILE_PATH})
file(WRITE ${VERSION_FILE_PATH} "Product Version=${OPENSIM_MAJOR_VERSION}.${OPENSIM_MINOR_VERSION}")
file(APPEND  ${VERSION_FILE_PATH} "\n")
file(APPEND  ${VERSION_FILE_PATH} "Compiler=${CMAKE_GENERATOR}-${CMAKE_CXX_COMPILER_ID}")
file(APPEND  ${VERSION_FILE_PATH} "\n")
file(APPEND  ${VERSION_FILE_PATH} "Platform=${PLATFORM_NAME}-${PLATFORM_ABI}")
file(APPEND  ${VERSION_FILE_PATH} "\n")
install(FILES ${VERSION_FILE_PATH} DESTINATION sdk)


# Preprocessor definitions.
# -------------------------
# These are used in OpenSim/version.h
set(OPENSIM_SYSTEM_INFO ${CMAKE_SYSTEM})
set(OPENSIM_OS_NAME ${CMAKE_SYSTEM_NAME})

if( WIN32 )
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
