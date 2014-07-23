PROJECT (OpenSim)
#
# Build of OpenSim. There are three steps:
#    (1) Choose appropriate platform
#    (2) Locate Simbody and its dependent libraries
#    (3) Build OpenSim libraries and executables
#    
#
#----------------------------------------------------

cmake_minimum_required(VERSION 2.6)
if(COMMAND cmake_policy)
        cmake_policy(SET CMP0003 NEW)
        cmake_policy(SET CMP0005 NEW)
endif(COMMAND cmake_policy)


SET(BUILD_BINARY_DIR ${CMAKE_BINARY_DIR}
    CACHE PATH 
    "The OpenSim build (not the install) puts all the libraries and executables together here (with /Release, etc. appended on some platforms).")

# Make everything go in the same binary directory. (These are CMake-defined
# variables.)
SET(EXECUTABLE_OUTPUT_PATH ${BUILD_BINARY_DIR})
SET(LIBRARY_OUTPUT_PATH ${BUILD_BINARY_DIR})

# Use this to generate a private set of libraries whose names
# won't conflict with installed versions.
SET(BUILD_USING_NAMESPACE "" CACHE STRING
	"All library names will be prefixed with 'xxx_' if this is set to xxx.")

SET(NameSpace)
IF(BUILD_USING_NAMESPACE)
    SET(NameSpace "${BUILD_USING_NAMESPACE}_")
ENDIF(BUILD_USING_NAMESPACE)

# Specify number of cores to run for build
SET(PROCESSOR_COUNT 8 CACHE STRING "Number of processors in the CPU")
MARK_AS_ADVANCED(PROCESSOR_COUNT)

# This will be initialized to the environment variable of the same name
# if it is set, otherwise it will be empty.
SET(SIMBODY_HOME $ENV{SIMBODY_HOME} CACHE
    PATH "The location of the Simbody installation to use; you can change this. Set as empty to let CMake search for Simbody automatically.")

#
# These are the names of all the libraries we depend on. These are
# target names so can be used to specify dependencies of one library
# on another. (In Debug mode the actual targets will have "_d" appended.)
#

SET(SimTKSIMBODY_LIBRARY_NAME ${NameSpace}SimTKsimbody CACHE STRING
"Base name of the library being built; can't be changed here; see BUILD_USING_NAMESPACE variable."
FORCE)
SET(SimTKMATH_LIBRARY_NAME ${NameSpace}SimTKmath CACHE STRING
"Base name of the library being built; can't be changed here; see BUILD_USING_NAMESPACE variable."
FORCE)
SET(SimTKCOMMON_LIBRARY_NAME ${NameSpace}SimTKcommon CACHE STRING
"Base name of the library being built; can't be changed here; see BUILD_USING_NAMESPACE variable."
FORCE)
MARK_AS_ADVANCED( SimTKSIMBODY_LIBRARY_NAME )
MARK_AS_ADVANCED( SimTKMATH_LIBRARY_NAME )
MARK_AS_ADVANCED( SimTKCOMMON_LIBRARY_NAME )


SET(SimTKCOMMON_SHARED_LIBRARY ${SimTKCOMMON_LIBRARY_NAME})
SET(SimTKMATH_SHARED_LIBRARY ${SimTKMATH_LIBRARY_NAME})
SET(SimTKSIMBODY_SHARED_LIBRARY ${SimTKSIMBODY_LIBRARY_NAME})

#
## Platform
#
# Create a platform name useful for finding things in the Platform
# directory.
IF(WIN32)
	IF(CMAKE_GENERATOR MATCHES  "Visual Studio 8")
		SET(VCVERSION VC8)
	ENDIF(CMAKE_GENERATOR MATCHES  "Visual Studio 8")
	IF(CMAKE_GENERATOR MATCHES  "Visual Studio 9")
		SET(VCVERSION VC9)
	ENDIF(CMAKE_GENERATOR MATCHES  "Visual Studio 9")
	IF(CMAKE_GENERATOR MATCHES  "Visual Studio 10")
		#NOTE: VC12 (VS2013) does not require explicit link line to Psapi library
		#Specifying here so it will link for VC10
		link_libraries(Psapi)
		SET(VCVERSION VC10)
	ENDIF(CMAKE_GENERATOR MATCHES  "Visual Studio 10")
	SET(Platform "Windows")
    SET(NATIVE_COPY_CMD copy)
ELSEIF(APPLE)
    SET(Platform Mac)
    SET(PLATFORM_NAME Mac)
    SET(NATIVE_COPY_CMD cp)
ELSE()
    SET(Platform Linux)
    SET(PLATFORM_NAME Linux)
    SET(NATIVE_COPY_CMD cp)
ENDIF()

# In addition to the platform name we need to know the Application Binary
# Interface (ABI) we're building for. Currently that is either x86, meaning
# 32 bit Intel instruction set, or x64 for 64 bit Intel instruction set.
# Kevin: Since Ubuntu 12 64bit libraries are in lib not lib64 (This in 
# response of Sherm's change on Simbody)

IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(PLATFORM_ABI x64)
	SET(ARCH64 ON) 
	#    SET(LIB64 64) Ubuntu 12.05
ELSE()
    SET(PLATFORM_ABI x86)
ENDIF()

SET(BUILD_PLATFORM "${PLATFORM_NAME}:${PLATFORM_ABI}" CACHE STRING
    "This is the platform and ABI we're building for. Not changeable here; use a different CMake generator instead."
    FORCE)

#######################################################################
# Allow build of binaries on Leopard that work on Tiger
# Plus, -m32 is required for building on Snow Leopard, until we have a 64-bit Mac Lapack
IF( APPLE )
   SET( CMAKE_CXX_FLAGS "-mmacosx-version-min=10.8 -m32" )
   SET( CMAKE_C_FLAGS "-mmacosx-version-min=10.8 -m32" )
   SET( LIB64  )
   SET(ARCH64 OFF) 
ENDIF( APPLE )

IF(UNIX AND NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release CACHE STRING "Debug or Release build" FORCE)
ENDIF (UNIX AND NOT CMAKE_BUILD_TYPE)

## Choose the maximum level of x86 instruction set that the compiler is 
## allowed to use. Was using sse2 but changed to let the compilers choose. Most
## will probably use sse2 or later by default.
## On 64 bit MSVC, the default is sse2 and the argument
## isn't recognized so we won't specify it.

if (CMAKE_CL_64)
    set(default_build_inst_set)
else()
    # Here's where we used to set sse2. Leaving this line in
    # case we decide to specify a default value again.
    set(default_build_inst_set)
endif()

## This can be set to a different value by the person running CMake.
SET(BUILD_INST_SET ""
    CACHE STRING 
    "CPU instruction level compiler is permitted to use (default: let compiler decide).")
MARK_AS_ADVANCED( BUILD_INST_SET )

if (BUILD_INST_SET)
    set(inst_set_to_use ${BUILD_INST_SET})
else()
    set(inst_set_to_use ${default_build_inst_set})
endif()

## When building in any of the Release modes, tell gcc to use not-quite most
## aggressive optimization and to generate SSE2 floating point instructions.
## Here we are specifying *all* of the Release flags, overriding CMake's
## defaults.  Watch out for optimizer bugs in particular gcc versions!

IF(${CMAKE_CXX_COMPILER_ID} MATCHES "GNU" OR
        ${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    if (inst_set_to_use)
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
    SET(GCC_OPT_ENABLE "-funroll-loops")

    # If you know of optimization bugs that affect SimTK in particular
    # gcc versions, this is the place to turn off those optimizations.
    SET(GCC_OPT_DISABLE)
    # We know Gcc 4.4.3 on Ubuntu 10 is buggy and that Snow Leopard's
    # 4.2.1 is not. To be safe for now we'll assume anything over 4.3
    # should have these disabled.
    if (GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3)
        SET(GCC_OPT_DISABLE 
	"-fno-tree-vrp -fno-strict-aliasing -fno-guess-branch-probability")
    endif()

    # C++
    SET(CMAKE_CXX_FLAGS_DEBUG          "-g ${GCC_INST_SET}" 
      CACHE STRING "g++ Debug build compile flags" FORCE)
    SET(CMAKE_CXX_FLAGS_RELEASE        
      "-DNDEBUG -O2 ${GCC_OPT_ENABLE} ${GCC_OPT_DISABLE} ${GCC_INST_SET}" 
      CACHE STRING "g++ Release build compile flags" FORCE)
    SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO 
      "-DNDEBUG -O2 -g ${GCC_OPT_ENABLE} ${GCC_OPT_DISABLE} ${GCC_INST_SET}" 
      CACHE STRING "g++ RelWithDebInfo build compile flags" FORCE)
    SET(CMAKE_CXX_FLAGS_MINSIZEREL     "-DNDEBUG -Os ${GCC_INST_SET}" 
      CACHE STRING "g++ MinSizeRel build compile flags" FORCE)

    # C
    SET(CMAKE_C_FLAGS_DEBUG            "-g ${GCC_INST_SET}" 
      CACHE STRING "gcc Debug build compile flags" FORCE)
    SET(CMAKE_C_FLAGS_RELEASE          
      "-DNDEBUG -O2 ${GCC_OPT_ENABLE} ${GCC_OPT_DISABLE} ${GCC_INST_SET}" 
      CACHE STRING "gcc Release build compile flags" FORCE)
    SET(CMAKE_C_FLAGS_RELWITHDEBINFO   
      "-DNDEBUG -O2 -g ${GCC_OPT_ENABLE} ${GCC_OPT_DISABLE} ${GCC_INST_SET}" 
      CACHE STRING "gcc RelWithDebInfo build compile flags" FORCE)
    SET(CMAKE_C_FLAGS_MINSIZEREL       "-DNDEBUG -Os ${GCC_INST_SET}" 
      CACHE STRING "gcc MinSizeRel build compile flags" FORCE)

ENDIF()

## When building in any of the Release modes, tell VC++ cl compiler to use intrinsics 
## (i.e. sqrt instruction rather than sqrt subroutine) with flag /Oi.

IF(WIN32 AND ${CMAKE_C_COMPILER} MATCHES "cl")
    if (inst_set_to_use)
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
    SET(CMAKE_CXX_FLAGS_DEBUG        
	"/MP${mxcpu} /D _DEBUG /MDd /Od /Ob0 /RTC1 /Zi /bigobj /GS- ${CL_INST_SET}" 
        CACHE STRING "VC++ Debug build compile flags" FORCE)
    SET(CMAKE_CXX_FLAGS_RELEASE        
	"/MP${mxcpu} /D NDEBUG /MD  /O2 /Ob2 /Oi /bigobj /GS- ${CL_INST_SET}" 
        CACHE STRING "VC++ Release build compile flags" FORCE)
    SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO 
	"/MP${mxcpu} /D NDEBUG /MD  /O2 /Ob2 /Oi /Zi /bigobj /GS- ${CL_INST_SET}" 
        CACHE STRING "VC++ RelWithDebInfo build compile flags" FORCE)
    SET(CMAKE_CXX_FLAGS_MINSIZEREL 
	"/MP${mxcpu} /D NDEBUG /MD  /O1 /Ob1 /Oi /bigobj /GS- ${CL_INST_SET}" 
        CACHE STRING "VC++ MinSizeRel build compile flags" FORCE)

    ## C
    SET(CMAKE_C_FLAGS_DEBUG        
	"/MP${mxcpu} /D _DEBUG /MDd /Od /Ob0 /RTC1 /Zi /GS- ${CL_INST_SET}" 
        CACHE STRING "VC++ Debug build compile flags" FORCE)
    SET(CMAKE_C_FLAGS_RELEASE        
	"/MP${mxcpu} /D NDEBUG /MD  /O2 /Ob2 /Oi /GS- ${CL_INST_SET}" 
        CACHE STRING "VC++ Release build compile flags" FORCE)
    SET(CMAKE_C_FLAGS_RELWITHDEBINFO 
	"/MP${mxcpu} /D NDEBUG /MD  /O2 /Ob2 /Oi /Zi /GS- ${CL_INST_SET}" 
        CACHE STRING "VC++ RelWithDebInfo build compile flags" FORCE)
    SET(CMAKE_C_FLAGS_MINSIZEREL 
	"/MP${mxcpu} /D NDEBUG /MD  /O1 /Ob1 /Oi /GS- ${CL_INST_SET}" 
        CACHE STRING "VC++ MinSizeRel build compile flags" FORCE)

ENDIF()

SET(BUILD_JAVA_WRAPPING OFF CACHE BOOL "Build Java wrapping (needed if you're building the GUI and have SWIG and Java installed on your machine.)")
MARK_AS_ADVANCED( BUILD_JAVA_WRAPPING )

SET(BUILD_PYTHON_WRAPPING OFF CACHE BOOL "Build Python wrapping (needed if you're building the Python wrapping and have SWIG and Python installed on your machine.)")
MARK_AS_ADVANCED( BUILD_PYTHON_WRAPPING )

# You can enable this advanced-variable feature to cause Visual Studio 
# or Makefiles to print the compile/link commands they are executing. Very
# useful for debugging CMake stuff but messy and obscures errors and warnings
# so should be off by default.
SET(CMAKE_VERBOSE_MAKEFILE OFF CACHE BOOL 
    "Enable this for verbose build output.")

IF(WIN32)
ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
ELSE(WIN32)
ENDIF(WIN32)

##################################################
## Define Platform to
##			Win32/VC${version}
##		or	Mac
##      or  Linux
##
## Also translate ARCH64 to platform specific flags
## 
##################################################
INCLUDE(InstallRequiredSystemLibraries)

IF(WIN32)
ELSE(WIN32)
	SET(NameSpace "")	## No renamed SimTK libraries except on Windows
	IF(APPLE)
	   IF(ARCH64) 
		   SET( CMAKE_CXX_FLAGS "-mmacosx-version-min=10.8" )
	   ELSE (ARCH64)
		   SET( CMAKE_CXX_FLAGS "-mmacosx-version-min=10.8 -m32" )
	   ENDIF (ARCH64)
	ELSE (APPLE)
	   IF(ARCH64) 
		  SET( CMAKE_CXX_FLAGS "-m64" )
	   ELSE (ARCH64)
		  # I don't believe setting the CXX flags is working in cmake.
		  SET( CMAKE_CXX_FLAGS "-m32" )
	   ENDIF(ARCH64)
	ENDIF (APPLE)
ENDIF(WIN32)

# C++11: In revision 8629, we started using std::unique_ptr, which requires
# C++11 features to be enabled when using GCC or Clang.
IF(${CMAKE_C_COMPILER} MATCHES "cc" OR ${CMAKE_C_COMPILER} MATCHES "clang")
    # Using C++11 on OSX requires using libc++ instead of libstd++.
    # libc++ is an implementation of the C++ standard library for OSX.
    IF(APPLE)
        IF(XCODE)
            SET(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
            SET(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
        ELSE()
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
        ENDIF()
    ELSE() # not APPLE
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    ENDIF()
ENDIF()

# Disable Build using the simtk headers and libraries checked in the repository. 
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

SET(BUILD_USING_NAMESPACE "OpenSim")

## Find Simbody.

# As of Simbody 3.4, Simbody has a SimbodyConfig.cmake file, which is a
# preferred way to find Simbody over the previous FindSimbody.cmake script.
# NO_MODULE means we will not allow the use of a FindSimbody.cmake script.
SET(SIMBODY_VERSION_TO_USE 3.5)

# Find Simbody freshly by unsetting this CMake-generated variable.
UNSET(Simbody_DIR CACHE)
IF("${SIMBODY_HOME}" STREQUAL "")
    # We assume the only case in which the user
    # wants us to search for Simbody is if they left SIMBODY_HOME empty.
    # If the user specifies an invalid SIMBODY_HOME by accident,
    # we shouldn't let that fail silently and still search for 
    # Simbody elsewhere; they may never realize
    # we are not using their requested installation of Simbody.
    FIND_PACKAGE(Simbody ${SIMBODY_VERSION_TO_USE} QUIET
        NO_MODULE)
ELSE()
    # Find the package using the user-specified path.
    # NO_DEFAULT_PATH will cause find_package to only 
    # look in the provided PATHS.
    FIND_PACKAGE(Simbody ${SIMBODY_VERSION_TO_USE} QUIET
        PATHS "${SIMBODY_HOME}" NO_MODULE NO_DEFAULT_PATH)
ENDIF()
# This variable appears in the CMake GUI and could confuse users,
# since this variable can't be changed manually.
MARK_AS_ADVANCED(Simbody_DIR)

# If Simbody is not found, Simbody_FOUND is false.
IF(NOT Simbody_FOUND)
    MESSAGE(FATAL_ERROR "
        Simbody ${SIMBODY_VERSION_TO_USE} not found. Install Simbody and set
        SIMBODY_HOME to the installation directory of Simbody.")
ENDIF()

include_directories(${Simbody_INCLUDE_DIR})
link_directories(${Simbody_LIB_DIR})

# MESSAGE("Simbody_LIB_DIR="${Simbody_LIB_DIR})
# MESSAGE("Simbody_BIN_DIR="${Simbody_BIN_DIR})
# MESSAGE("Simbody_INCLUDE_DIR="${Simbody_INCLUDE_DIR})

# Determine which math libraries to use for this platform.
SET(BUILD_USING_OTHER_LAPACK "" CACHE STRING
 "If you have your own Lapack, put its library basename here. Default is to use lapack.")

set(default_lapack_libs "lapack;blas")
if (WIN32) 
         set(default_lapack_libs "liblapack;libblas")
endif()
SET(LAPACK_BEING_USED ${default_lapack_libs} CACHE STRING
"Basename of the actual Lapack library we're depending on; can't change here; see variable BUILD_USING_OTHER_LAPACK." FORCE)

IF(BUILD_USING_OTHER_LAPACK)
    SET(LAPACK_BEING_USED ${BUILD_USING_OTHER_LAPACK} CACHE STRING
"Basename of the actual Lapack library we're depending on; can't change here; see variable BUILD_USING_OTHER_LAPACK." FORCE)
ENDIF(BUILD_USING_OTHER_LAPACK)

#IF(${CMAKE_C_COMPILER} MATCHES "cl")
    ## Assume Microsoft Visual Studio
	#    SET(MATH_LIBS_TO_USE    ${LAPACK_BEING_USED} pthreadVC2)
	#ELSE(${CMAKE_C_COMPILER} MATCHES "cl")
	#    IF(APPLE)
	#    SET(REALTIME_LIB)
	# ELSE()
	#    SET(REALTIME_LIB rt)
	# ENDIF()
	#SET(MATH_LIBS_TO_USE    ${LAPACK_BEING_USED} pthread ${REALTIME_LIB} dl)
	#ENDIF(${CMAKE_C_COMPILER} MATCHES "cl")

IF(${CMAKE_C_COMPILER} MATCHES "cc" OR ${CMAKE_C_COMPILER} MATCHES "clang")
    IF(APPLE)
	    SET(REALTIME_LIB)
    ELSE()
	    SET(REALTIME_LIB rt)
    ENDIF()
    SET(MATH_LIBS_TO_USE    ${LAPACK_BEING_USED} pthread ${REALTIME_LIB} dl)
    SET(MATH_LIBS_TO_USE_VN ${LAPACK_BEING_USED} pthread ${REALTIME_LIB} dl)
ELSE()
    ## Assume Microsoft Visual Studio
    if(${PLATFORM_ABI} MATCHES "x64")
        SET(MATH_LIBS_TO_USE    ${LAPACK_BEING_USED} pthreadVC2_x64)
        SET(MATH_LIBS_TO_USE_VN ${LAPACK_BEING_USED} pthreadVC2_x64)
    else()
        SET(MATH_LIBS_TO_USE    ${LAPACK_BEING_USED} pthreadVC2)
        SET(MATH_LIBS_TO_USE_VN ${LAPACK_BEING_USED} pthreadVC2)
    endif()
ENDIF()
# Copy libraries from ${SIMTK_BIN_DIR} to ${BUILD_BINARY_DIR}

IF(WIN32)
    FILE(GLOB LIB_FILES
        "${Simbody_BIN_DIR}/*.dll"
        "${Simbody_BIN_DIR}/*") # .dll, .exe, no suffix executable
ELSE()
    FILE(GLOB LIB_FILES
        "${Simbody_LIB_DIR}/*SimTK*.a"
        "${Simbody_LIB_DIR}/*SimTK*.so"
        "${Simbody_LIB_DIR}/*SimTK*.so.*"
        "${Simbody_LIB_DIR}/*SimTK*.dylib"
        )
ENDIF()

# This is where we're going to put these binaries.
SET(COPIED_LIB_FILES)
FOREACH(LIBF ${LIB_FILES})
	GET_FILENAME_COMPONENT(LIBF_ROOT ${LIBF} NAME)
	SET(COPIED_LIB_FILES ${COPIED_LIB_FILES}
	    "${BUILD_BINARY_DIR}/${CMAKE_CFG_INTDIR}/${LIBF_ROOT}")
ENDFOREACH()

# This target depends on the destination copies of the Simbody files (in the
# binary build directory).  Those are produced by the OUTPUT custom command 
# below, which depends on the source files from the Simbody installation 
# directory.
ADD_CUSTOM_TARGET(SimbodyFiles ALL DEPENDS ${COPIED_LIB_FILES})
SET_TARGET_PROPERTIES(SimbodyFiles
 	PROPERTIES PROJECT_LABEL "Library - Simbody Files")

FOREACH(LIBF ${LIB_FILES})
    # MESSAGE("LIB_FILE" ${LIBF})
	GET_FILENAME_COMPONENT(LIBF_ROOT ${LIBF} NAME)
	GET_FILENAME_COMPONENT(LIBF_SUFFIX ${LIBF} EXT)
	SET(COPIED_LIBF "${BUILD_BINARY_DIR}/${CMAKE_CFG_INTDIR}/${LIBF_ROOT}")
	FILE(TO_NATIVE_PATH "${LIBF}" LIBF_SRC)
	FILE(TO_NATIVE_PATH "${COPIED_LIBF}" LIBF_DEST)

    # Copy Simbody files if they are out of date. This is invoked because
    # the SimbodyFiles target depends on these output files.
	ADD_CUSTOM_COMMAND(OUTPUT "${COPIED_LIBF}"
		COMMAND ${NATIVE_COPY_CMD} "${LIBF_SRC}" "${LIBF_DEST}"
		DEPENDS "${LIBF}"
		COMMENT "Copy ${LIBF_SRC} -> ${LIBF_DEST}"
		VERBATIM)
ENDFOREACH()

INSTALL(DIRECTORY ${Simbody_INCLUDE_DIR} DESTINATION sdk/include/SimTK 
			PATTERN ".svn" EXCLUDE)
			

IF(NOT LIBRARY_OUTPUT_PATH)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/${VCVERSION} CACHE INTERNAL "Single output directory for building all libraries.")
ENDIF(NOT LIBRARY_OUTPUT_PATH)
IF(NOT EXECUTABLE_OUTPUT_PATH)
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/${VCVERSION} CACHE INTERNAL "Single output directory for building all executables.")
ENDIF(NOT EXECUTABLE_OUTPUT_PATH)
SET(OpenSim_LIBRARY_DIR ${LIBRARY_OUTPUT_PATH})
SET(OpenSim_EXECUTABLE_DIR ${EXECUTABLE_OUTPUT_PATH})

# Other than Windows we can debug without debuggable SimTK libraries
IF(WIN32)
SET(CMAKE_DEBUG_POSTFIX "_d" CACHE INTERNAL "" FORCE)
ELSE(WIN32)
SET(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "Suffix for debug libraries")
ENDIF(WIN32)

#
INCLUDE_DIRECTORIES(${Simbody_INCLUDE_DIR})

IF(WIN32)
		LINK_DIRECTORIES(${Simbody_LIB_DIR} ${OpenSim_LIBRARY_DIR})
ELSE(WIN32)
	IF(APPLE)
		LINK_DIRECTORIES(${Simbody_LIB_DIR} ${OpenSim_LIBRARY_DIR})
	ELSE(APPLE) # Linux?
		LINK_DIRECTORIES(${Simbody_LIB_DIR}/ ${OpenSim_LIBRARY_DIR})
	ENDIF(APPLE)
ENDIF(WIN32)

SET(SIMTK_COMMON_LIB debug ${NameSpace}SimTKcommon${CMAKE_DEBUG_POSTFIX} optimized ${NameSpace}SimTKcommon)
SET(SIMTK_MATH_LIB debug ${NameSpace}SimTKmath${CMAKE_DEBUG_POSTFIX} optimized ${NameSpace}SimTKmath)
SET(SIMTK_SIMBODY_LIB debug ${NameSpace}SimTKsimbody${CMAKE_DEBUG_POSTFIX} optimized ${NameSpace}SimTKsimbody)

SET(SIMTK_ALL_LIBS	${SIMTK_COMMON_LIB} 
                    ${SIMTK_MATH_LIB} 
                    ${SIMTK_SIMBODY_LIB} 
                    ${MATH_LIBS_TO_USE})

# Now figure out which 3rd party files need to be installed in the /bin 
# directory and which in the lib directory. Executables go in /bin on all 
# platforms.  Libraries (shared or otherwise) go in /lib, except on Windows 
# where the ".dll" files go in bin along with executables. Anything that
# is only needed for programmers goes in sdk/lib.
IF(WIN32)	
	FILE(GLOB THIRDPARTY_BIN_FILES 
					${Simbody_BIN_DIR}/*.dll	# non repository layout
					${Simbody_BIN_DIR}/*.exe
					${Simbody_BIN_DIR}/${VCVERSION}/*.dll
					${Simbody_BIN_DIR}/${VCVERSION}/*.exe)
	FILE(GLOB THIRDPARTY_SDKLIB_FILES 
					${Simbody_LIB_DIR}/*.lib	# non repository layout
					${Simbody_LIB_DIR}/${VCVERSION}/*.lib)
ELSE(WIN32)
	FILE(GLOB THIRDPARTY_BIN_FILES 
        ${Simbody_BIN_DIR}/*) # executables only (e.g. visualizer)
    # If the LIB_DIR is some common place for libraries (e.g., /usr/lib),
    # we only want to copy over the libraries that have SimTK in the name.
	FILE(GLOB THIRDPARTY_LIB_FILES 
        ${Simbody_LIB_DIR}/*SimTK*.so
        ${Simbody_LIB_DIR}/*SimTK*.so.*
        ${Simbody_LIB_DIR}/*SimTK*.dylib)
	FILE(GLOB THIRDPARTY_SDKLIB_FILES 
        ${Simbody_LIB_DIR}/*SimTK*.a)
ENDIF(WIN32)

# Specify "PROGRAMS" rather than "FILES" so the execute bit gets set.
INSTALL(PROGRAMS ${THIRDPARTY_BIN_FILES}    
        DESTINATION bin)
INSTALL(FILES ${THIRDPARTY_LIB_FILES}    DESTINATION lib)
INSTALL(FILES ${THIRDPARTY_SDKLIB_FILES} DESTINATION sdk/lib)


## The following are required to uses Dart and the Cdash dashboard per Jesse
ENABLE_TESTING()
INCLUDE(CTest)

# Sets the number of CPUs testing would use
SET (cmd ${CMAKE_CTEST_COMMAND} -j${PROCESSOR_COUNT})
IF (MSVC)
    SET (cmd ${cmd} -C ${CMAKE_CFG_INTDIR})
ELSE (MSVC)
    SET (cmd ${cmd} -C ${CMAKE_BUILD_TYPE})
ENDIF (MSVC)
ADD_CUSTOM_TARGET (RUN_TESTS_PARALLEL
    COMMAND ${cmd}
)

## This adds a target "doxygen" for building the doxygen docs, as well as 
## an INSTALL procedure.
INCLUDE(ApiDoxygen.cmake)

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "OpenSim project")
SET(CPACK_PACKAGE_VENDOR "Stanford University, National Center for Biomedical Computation")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ReadMe.txt")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/Copyright.txt")
SET(CPACK_PACKAGE_VERSION_MAJOR "3")
SET(CPACK_PACKAGE_VERSION_MINOR "2")
SET(CPACK_PACKAGE_VERSION_PATCH "0")
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "OpenSim ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}")

# Create buildinfo.txt file and place under sdk to include product version, platform and compiler for troubleshooting purposes
SET(VERSION_FILE_PATH ${CMAKE_BINARY_DIR}/buildinfo.txt)
# MESSAGE("version file="${VERSION_FILE_PATH})
FILE(WRITE ${VERSION_FILE_PATH} "Product Version=${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}")
FILE(APPEND  ${VERSION_FILE_PATH} "\n")
FILE(APPEND  ${VERSION_FILE_PATH} "Compiler=${CMAKE_GENERATOR}-${CMAKE_CXX_COMPILER_ID}")
FILE(APPEND  ${VERSION_FILE_PATH} "\n")
FILE(APPEND  ${VERSION_FILE_PATH} "Platform=${Platform}-${PLATFORM_ABI}")
FILE(APPEND  ${VERSION_FILE_PATH} "\n")
INSTALL(FILES ${VERSION_FILE_PATH} DESTINATION sdk)

#
# Build the documentation
#

#-----------------------------------------------------------------------------
SET(BUILD_API_ONLY OFF CACHE BOOL "Build/install only headers, libraries,
    wrapping, tests; not applications (ik, rra, etc.).")
MARK_AS_ADVANCED( BUILD_API_ONLY )

IF(BUILD_API_ONLY)
	SUBDIRS(Vendors OpenSim)
ELSE(BUILD_API_ONLY)
	SUBDIRS(Vendors OpenSim Applications)
ENDIF(BUILD_API_ONLY)

