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

# Decide if NameSpace for Simbody and other libraries is assumed (Prefix, defaults to OpenSim_ on Windows and empty otherwise)
IF(WIN32)
	SET(NameSpace "OpenSim_" CACHE STRING "Prefix for simtk lib names, includes trailing '_'. Set to '' to use stock SimTK libraries.")
ELSE(WIN32)
	SET(NameSpace "" CACHE STRING "Prefix for simtk lib names, includes trailing '_'. Set to '' to use stock SimTK libraries.")
ENDIF(WIN32)

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


SET(SimTKCOMMON_SHARED_LIBRARY ${SimTKCOMMON_LIBRARY_NAME})
SET(SimTKCOMMON_STATIC_LIBRARY ${SimTKCOMMON_LIBRARY_NAME}_static)
SET(SimTKMATH_SHARED_LIBRARY ${SimTKMATH_LIBRARY_NAME})
SET(SimTKMATH_STATIC_LIBRARY ${SimTKMATH_LIBRARY_NAME}_static)
SET(SimTKSIMBODY_SHARED_LIBRARY ${SimTKSIMBODY_LIBRARY_NAME})
SET(SimTKSIMBODY_STATIC_LIBRARY ${SimTKSIMBODY_LIBRARY_NAME}_static)

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
	SET(Platform "Windows")
    SET(NATIVE_COPY_CMD copy)
ELSEIF(APPLE)
    SET(PLATFORM_NAME Mac)
    SET(NATIVE_COPY_CMD cp)
ELSE()
    SET(PLATFORM_NAME Linux)
    SET(NATIVE_COPY_CMD cp)
ENDIF()

# In addition to the platform name we need to know the Application Binary
# Interface (ABI) we're building for. Currently that is either x86, meaning
# 32 bit Intel instruction set, or x64 for 64 bit Intel instruction set.

IF(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
    SET(PLATFORM_ABI x64)
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
   SET( CMAKE_CXX_FLAGS "-mmacosx-version-min=10.4 -m32" )
   SET( CMAKE_C_FLAGS "-mmacosx-version-min=10.4 -m32" )
   SET( LIB64  )
ENDIF( APPLE )

IF(UNIX AND NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release CACHE STRING "Debug or Release build" FORCE)
ENDIF (UNIX AND NOT CMAKE_BUILD_TYPE)

## Choose the maximum level of x86 instruction set that the compiler is 
## allowed to use. SSE2 is ubiquitous enough now that we don't mind
## abandoning machines that can't handle those instructions. SSE3 migh
## also be reasonable by now (April 2009) so this default should be
## revisited soon. This can be set to a different value by the person
## running CMake.
SET(BUILD_INST_SET "sse2"  # use SSE2 instruction set by default
    CACHE STRING "CPU instruction level compiler is permitted to use.")
MARK_AS_ADVANCED( BUILD_INST_SET )

## When building in any of the Release modes, tell gcc to use full optimization and
## to generate SSE2 floating point instructions. Here we are specifying *all* of the
## Release flags, overriding CMake's defaults.
## Watch out for optimizer bugs in particular gcc versions!

IF(${CMAKE_C_COMPILER} MATCHES "gcc")
    string(TOLOWER ${BUILD_INST_SET} GCC_INST_SET)

    # Get the gcc version number in major.minor.build format
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion
                    OUTPUT_VARIABLE GCC_VERSION)

    # Unrolling fixed-count loops is a useful optimization for Simmatrix.
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
    SET(CMAKE_CXX_FLAGS_DEBUG          "-g -m${GCC_INST_SET}" 
      CACHE STRING "g++ Debug build compile flags" FORCE)
    SET(CMAKE_CXX_FLAGS_RELEASE        
      "-DNDEBUG -O3 ${GCC_OPT_ENABLE} ${GCC_OPT_DISABLE} -m${GCC_INST_SET}" 
      CACHE STRING "g++ Release build compile flags" FORCE)
    SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO 
      "-DNDEBUG -O3 -g ${GCC_OPT_ENABLE} ${GCC_OPT_DISABLE} -m${GCC_INST_SET}" 
      CACHE STRING "g++ RelWithDebInfo build compile flags" FORCE)
    SET(CMAKE_CXX_FLAGS_MINSIZEREL     "-DNDEBUG -Os -m${GCC_INST_SET}" 
      CACHE STRING "g++ MinSizeRel build compile flags" FORCE)

    # C
    SET(CMAKE_C_FLAGS_DEBUG            "-g -m${GCC_INST_SET}" 
      CACHE STRING "gcc Debug build compile flags" FORCE)
    SET(CMAKE_C_FLAGS_RELEASE          
      "-DNDEBUG -O3 ${GCC_OPT_ENABLE} ${GCC_OPT_DISABLE} -m${GCC_INST_SET}" 
      CACHE STRING "gcc Release build compile flags" FORCE)
    SET(CMAKE_C_FLAGS_RELWITHDEBINFO   
      "-DNDEBUG -O3 -g ${GCC_OPT_ENABLE} ${GCC_OPT_DISABLE} -m${GCC_INST_SET}" 
      CACHE STRING "gcc RelWithDebInfo build compile flags" FORCE)
    SET(CMAKE_C_FLAGS_MINSIZEREL       "-DNDEBUG -Os -m${GCC_INST_SET}" 
      CACHE STRING "gcc MinSizeRel build compile flags" FORCE)

ENDIF(${CMAKE_C_COMPILER} MATCHES "gcc")

## When building in any of the Release modes, tell VC++ cl compiler to use intrinsics 
## (i.e. sqrt instruction rather than sqrt subroutine) with flag /Oi.

IF(${CMAKE_C_COMPILER} MATCHES "cl")
    STRING(TOUPPER ${BUILD_INST_SET} CL_INST_SET)

    ## C++
    SET(CMAKE_CXX_FLAGS_DEBUG        
	"/D _DEBUG /MDd /Od /Ob0 /RTC1 /Zi /GS- /arch:${CL_INST_SET}" 
        CACHE STRING "VC++ Debug build compile flags" FORCE)
    SET(CMAKE_CXX_FLAGS_RELEASE        
	"/D NDEBUG /MD  /O2 /Ob2 /Oi /GS- /arch:${CL_INST_SET}" 
        CACHE STRING "VC++ Release build compile flags" FORCE)
    SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO 
	"/D NDEBUG /MD  /O2 /Ob2 /Oi /Zi /GS- /arch:${CL_INST_SET}" 
        CACHE STRING "VC++ RelWithDebInfo build compile flags" FORCE)
    SET(CMAKE_CXX_FLAGS_MINSIZEREL 
	"/D NDEBUG /MD  /O1 /Ob1 /Oi /GS- /arch:${CL_INST_SET}" 
        CACHE STRING "VC++ MinSizeRel build compile flags" FORCE)

    ## C
    SET(CMAKE_C_FLAGS_DEBUG        
	"/D _DEBUG /MDd /Od /Ob0 /RTC1 /Zi /GS- /arch:${CL_INST_SET}" 
        CACHE STRING "VC++ Debug build compile flags" FORCE)
    SET(CMAKE_C_FLAGS_RELEASE        
	"/D NDEBUG /MD  /O2 /Ob2 /Oi /GS- /arch:${CL_INST_SET}" 
        CACHE STRING "VC++ Release build compile flags" FORCE)
    SET(CMAKE_C_FLAGS_RELWITHDEBINFO 
	"/D NDEBUG /MD  /O2 /Ob2 /Oi /Zi /GS- /arch:${CL_INST_SET}" 
        CACHE STRING "VC++ RelWithDebInfo build compile flags" FORCE)
    SET(CMAKE_C_FLAGS_MINSIZEREL 
	"/D NDEBUG /MD  /O1 /Ob1 /Oi /GS- /arch:${CL_INST_SET}" 
        CACHE STRING "VC++ MinSizeRel build compile flags" FORCE)

ENDIF(${CMAKE_C_COMPILER} MATCHES "cl")

SET(BUILD_JAVA_WRAPPING OFF CACHE BOOL "Build Java wrapping (needed if you're building the GUI and have SWIG and Java installed on your machine.)")


# Uncomment to make the linux Makefiles print the compile/link commands they are executing
SET(CMAKE_VERBOSE_MAKEFILE ON)

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

IF(WIN32)
ELSE(WIN32)
	SET(NameSpace "")	## No renamed SimTK libraries except on Windows
	IF(APPLE)
	   SET(Platform "Mac")
	   IF(ARCH64) 
		   SET( CMAKE_CXX_FLAGS "-mmacosx-version-min=10.4" )
	   ELSE (ARCH64)
		   SET( CMAKE_CXX_FLAGS "-mmacosx-version-min=10.4 -m32" )
	   ENDIF (ARCH64)
	ELSE (APPLE)
	   SET(Platform "Linux")
	   IF(ARCH64) 
	   ELSE (ARCH64)
		  # I don't believe setting the CXX flags is working in cmake.
		  SET( CMAKE_CXX_FLAGS "-m32" )
	   ENDIF(ARCH64)
	ENDIF (APPLE)
ENDIF(WIN32)

## When building in any of the Release modes, tell gcc to use full optimization and
## to generate SSE2 floating point instructions. Here we are specifying *all* of the
## Release flags, overriding CMake's defaults.

IF(${CMAKE_C_COMPILER} MATCHES "gcc")
    # C++
    SET(CMAKE_CXX_FLAGS_DEBUG          "-g  ${GCC_ARCH_FLAG}" 
        CACHE STRING "g++ Debug build compile flags" FORCE)
    SET(CMAKE_CXX_FLAGS_RELEASE        "-DNDEBUG -O3  ${GCC_ARCH_FLAG}" 
        CACHE STRING "g++ Release build compile flags" FORCE)
    SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-DNDEBUG -O3 -g  ${GCC_ARCH_FLAG}" 
        CACHE STRING "g++ RelWithDebInfo build compile flags" FORCE)
    SET(CMAKE_CXX_FLAGS_MINSIZEREL     "-DNDEBUG -Os  ${GCC_ARCH_FLAG}" 
        CACHE STRING "g++ MinSizeRel build compile flags" FORCE)

    # C
    SET(CMAKE_C_FLAGS_DEBUG            "-g  ${GCC_ARCH_FLAG}" 
        CACHE STRING "gcc Debug build compile flags" FORCE)
    SET(CMAKE_C_FLAGS_RELEASE          "-DNDEBUG -O3  ${GCC_ARCH_FLAG}" 
        CACHE STRING "gcc Release build compile flags" FORCE)
    SET(CMAKE_C_FLAGS_RELWITHDEBINFO   "-DNDEBUG -O3 -g  ${GCC_ARCH_FLAG}" 
        CACHE STRING "gcc RelWithDebInfo build compile flags" FORCE)
    SET(CMAKE_C_FLAGS_MINSIZEREL       "-DNDEBUG -Os  ${GCC_ARCH_FLAG}" 
        CACHE STRING "gcc MinSizeRel build compile flags" FORCE)

ENDIF(${CMAKE_C_COMPILER} MATCHES "gcc")


# Build using the simtk headers and libraries checked in the repository. 
# Currently this only works on windows VC8 since these are the only libraries checked in

SET(USE_LIBRARIES_FROM_REPOSITORY OFF CACHE BOOL "Use stock SimTK libraries based on SimtkCore 2.2")

IF(WIN32)
	SET(CMAKE_INSTALL_PREFIX "${PROGFILEDIR}/OpenSimDev" CACHE PATH "Install location to place headers and libraries")
	SET(SIMTK_DIR "${PROGFILEDIR}/SimTK/" CACHE PATH "Top-level directory of SimTK install")
ELSE(WIN32)
	IF(NOT CMAKE_INSTALL_PREFIX)
		SET(CMAKE_INSTALL_PREFIX "/usr/local/OpenSim")
	ENDIF(NOT CMAKE_INSTALL_PREFIX)
	SET(SIMTK_DIR "/usr/local/SimTK" CACHE PATH "Top-level directory of SimTK install")
ENDIF(WIN32)

# Determine which math libraries to use for this platform.
SET(BUILD_USING_OTHER_LAPACK "" CACHE STRING
 "If you have your own Lapack, put its library basename here. Default is to use SimTKlapack.")

SET(LAPACK_BEING_USED SimTKlapack CACHE STRING
"Basename of the actual Lapack library we're depending on; can't change here; see variable BUILD_USING_OTHER_LAPACK." FORCE)

IF(BUILD_USING_OTHER_LAPACK)
    SET(LAPACK_BEING_USED ${BUILD_USING_OTHER_LAPACK} CACHE STRING
"Basename of the actual Lapack library we're depending on; can't change here; see variable BUILD_USING_OTHER_LAPACK." FORCE)
ENDIF(BUILD_USING_OTHER_LAPACK)

IF(${CMAKE_C_COMPILER} MATCHES "gcc")
    IF(APPLE)
	    SET(REALTIME_LIB)
    ELSE()
	    SET(REALTIME_LIB rt)
    ENDIF()
    SET(MATH_LIBS_TO_USE    ${LAPACK_BEING_USED} pthread ${REALTIME_LIB} dl)
ELSE(${CMAKE_C_COMPILER} MATCHES "gcc")
    ## Assume Microsoft Visual Studio
    SET(MATH_LIBS_TO_USE    ${LAPACK_BEING_USED} pthreadVC2)
ENDIF(${CMAKE_C_COMPILER} MATCHES "gcc")

# If using prebuilt libraries reset SIMTK_DIR to relative path in source tree
IF(USE_LIBRARIES_FROM_REPOSITORY)
	SET(SIMTK_INCLUDE_DIR ${OpenSim_SOURCE_DIR}/Vendors/SimTK/include ${OpenSim_SOURCE_DIR}/Vendors/SimTK/${Platform}/include)
	SET(SIMTK_LIB_DIR ${OpenSim_SOURCE_DIR}/Vendors/SimTK/${Platform})
	SET(SIMTK_BIN_DIR ${SIMTK_LIB_DIR})
ELSE(USE_LIBRARIES_FROM_REPOSITORY)
	SET(SIMTK_INCLUDE_DIR ${SIMTK_DIR}/include)
	SET(SIMTK_LIB_DIR ${SIMTK_DIR}/lib)	
	SET(SIMTK_BIN_DIR ${SIMTK_DIR}/bin)	
ENDIF(USE_LIBRARIES_FROM_REPOSITORY)

INSTALL(DIRECTORY ${SIMTK_INCLUDE_DIR} DESTINATION sdk/include/SimTK 
			PATTERN ".svn" EXCLUDE)

IF(WIN32 AND USE_LIBRARIES_FROM_REPOSITORY)

ENDIF(WIN32 AND USE_LIBRARIES_FROM_REPOSITORY)

SET(OPENSIM_INSTALL_PACKAGE_BUILD OFF CACHE BOOL "Build an installer (to post on simtk.org)")
IF(OPENSIM_INSTALL_PACKAGE_BUILD)
IF(WIN32)
	SET(NETBEANS_INSTALL_DIR "C:/Program Files/netbeans-5.5/" CACHE PATH "Netbeans install directory")
	SET(VTK_LIB_DIR ${OpenSim_SOURCE_DIR}/Vendors/vtk_dll CACHE PATH "Prebuilt VTK-shared libraries")	
	SET(JRE_TO_PACKAGE "C:/Program Files/Java/jdk1.5.0_6/jre" CACHE PATH "JRE to package with OpenSim (jre dir)")
ENDIF(WIN32)
ENDIF(OPENSIM_INSTALL_PACKAGE_BUILD)

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
SET(CMAKE_DEBUG_POSTFIX "" CACHE STRING "Suffix for debug libraries")
ENDIF(WIN32)

# SET(CMAKE_INSTALL_PREFIX ${SIMTK_OPENSIM_INSTALL_DIR})

SET(XERCES_DIR_NAME xerces-c_2_8_0)
SET(XERCES_INCLUDE_DIR ${OpenSim_SOURCE_DIR}/Vendors/${XERCES_DIR_NAME}/include)

IF(WIN32 AND USE_LIBRARIES_FROM_REPOSITORY)
   INCLUDE_DIRECTORIES(${SIMTK_INCLUDE_DIR} ${SIMTK_INCLUDE_DIR}/${Platform} ${XERCES_INCLUDE_DIR})
ELSE(WIN32 AND USE_LIBRARIES_FROM_REPOSITORY)
   INCLUDE_DIRECTORIES(${SIMTK_INCLUDE_DIR} ${XERCES_INCLUDE_DIR})
ENDIF(WIN32 AND USE_LIBRARIES_FROM_REPOSITORY)

IF(WIN32)
	LINK_DIRECTORIES(${SIMTK_LIB_DIR}/${VCVERSION} ${OpenSim_LIBRARY_DIR} ${OpenSim_SOURCE_DIR}/Vendors/${XERCES_DIR_NAME}/lib/${Platform}/${VCVERSION})
ELSE(WIN32)
	IF(APPLE)
		LINK_DIRECTORIES(${SIMTK_LIB_DIR} ${OpenSim_LIBRARY_DIR} ${OpenSim_SOURCE_DIR}/Vendors/${XERCES_DIR_NAME}/lib${LIB64}/${Platform})
	ELSE(APPLE) # Linux?
		LINK_DIRECTORIES(${SIMTK_DIR}/lib${LIB64} ${OpenSim_LIBRARY_DIR} ${OpenSim_SOURCE_DIR}/Vendors/${XERCES_DIR_NAME}/lib${LIB64}/${Platform})
	ENDIF(APPLE)
ENDIF(WIN32)

SET(SIMTK_COMMON_LIB debug ${NameSpace}SimTKcommon${CMAKE_DEBUG_POSTFIX} optimized ${NameSpace}SimTKcommon)
SET(SIMTK_MATH_LIB debug ${NameSpace}SimTKmath${CMAKE_DEBUG_POSTFIX} optimized ${NameSpace}SimTKmath)
SET(SIMTK_SIMBODY_LIB debug ${NameSpace}SimTKsimbody${CMAKE_DEBUG_POSTFIX} optimized ${NameSpace}SimTKsimbody)

IF(WIN32)
SET(SIMTK_ALL_LIBS	${SIMTK_COMMON_LIB} 
					${SIMTK_MATH_LIB} 
					${SIMTK_SIMBODY_LIB} 
					${MATH_LIBS_TO_USE})
FILE(GLOB THIRDPARTY_DYNLIB_FILES 
							${SIMTK_BIN_DIR}/*.*
							${SIMTK_LIB_DIR}/*.*
							${OpenSim_SOURCE_DIR}/Vendors/${XERCES_DIR_NAME}/lib/${Platform}/xerces-c_2_8.dll)

ELSE(WIN32)
SET(SIMTK_ALL_LIBS	${SIMTK_COMMON_LIB} 
					${SIMTK_MATH_LIB} 
					${SIMTK_SIMBODY_LIB} 
					${MATH_LIBS_TO_USE}
					)
SET(THIRDPARTY_DYNLIB_FILES	${SIMTK_LIB_DIR}/lib${NameSpace}SimTKcommon${SHARED_LIB_EXT} 
						${SIMTK_LIB_DIR}/lib${NameSpace}SimTKmath${SHARED_LIB_EXT} 
						${SIMTK_LIB_DIR}/lib${NameSpace}SimTKsimbody${SHARED_LIB_EXT}
						${SIMTK_LIB_DIR}/libSimTKlapack${SHARED_LIB_EXT}
						${OpenSim_SOURCE_DIR}/Vendors/${XERCES_DIR_NAME}/lib${LIB64}/${Platform}/libxerces-c${SHARED_LIB_EXT})					
ENDIF(WIN32)

# We don't really want to copy the version of lapack we're using because that may be specific to our machine, so we take the generic lapack version (i.e. generic pentium 32 bit single proc)
IF(WIN32)
INSTALL_FILES(/bin/ FILES ${THIRDPARTY_DYNLIB_FILES})
INSTALL_FILES(/sdk/lib/ FILES ${THIRDPARTY_EXPORTLIB_FILES})

	IF(OPENSIM_INSTALL_PACKAGE_BUILD)
	# Install windows runtime libraries to support prebuilt libraries and redistributable
	INSTALL_FILES(/bin/ FILES 
					${OpenSim_SOURCE_DIR}/Vendors/vtk_dll/msvcr71.dll
		            ${OpenSim_SOURCE_DIR}/Vendors/vtk_dll/msvcp71.dll
		            ${SIMTK_LIB_DIR}/msvcr80.dll
		            ${SIMTK_LIB_DIR}/msvcp80.dll
		            ${SIMTK_LIB_DIR}/vcredist_x86.exe)
	ENDIF(OPENSIM_INSTALL_PACKAGE_BUILD)

ELSE(WIN32)
		FILE(GLOB XERCES_LIBS ${OpenSim_SOURCE_DIR}/Vendors/${XERCES_DIR_NAME}/lib/${Platform}/libx*.*)
	IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
		INSTALL_FILES(/bin/ FILES ${SIMTK_BIN_DIR}/lib${NameSpace}SimTKcommon${CMAKE_DEBUG_POSTFIX}${SHARED_LIB_EXT} 
									${SIMTK_BIN_DIR}/lib${NameSpace}SimTKmath${CMAKE_DEBUG_POSTFIX}${SHARED_LIB_EXT} 
									${SIMTK_BIN_DIR}/libSimTKlapack${CMAKE_DEBUG_POSTFIX}${SHARED_LIB_EXT}
									${SIMTK_BIN_DIR}/lib${NameSpace}SimTKsimbody${CMAKE_DEBUG_POSTFIX}${SHARED_LIB_EXT}
									${XERCES_LIBS}
									${SIMTK_BIN_DIR}/${ATLAS_LIB}${SHARED_LIB_EXT})
	ELSE(CMAKE_BUILD_TYPE STREQUAL "Debug")
		INSTALL_FILES(/sdk/lib/ FILES ${SIMTK_LIB_DIR}/lib${NameSpace}SimTKcommon${SHARED_LIB_EXT} 
									${SIMTK_LIB_DIR}/lib${NameSpace}SimTKmath${SHARED_LIB_EXT} 
									${SIMTK_LIB_DIR}/libSimTKlapack${SHARED_LIB_EXT}
									${SIMTK_LIB_DIR}/lib${NameSpace}SimTKsimbody${SHARED_LIB_EXT}
									${XERCES_LIBS}
									${SIMTK_LIB_DIR}/${ATLAS_LIB}${SHARED_LIB_EXT})
	ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")
ENDIF(WIN32)


## Install other files (batch file, doc, api)
INCLUDE (Dart)

## Docs
SET(INSTALL_DOC OFF CACHE BOOL "Install Doxygen generated files into distribution")
##IF(INSTALL_DOC)
##	INCLUDE(ApiDoxygen.cmake)
##ENDIF(INSTALL_DOC)

IF(OPENSIM_INSTALL_PACKAGE_BUILD)
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "OpenSim project")
SET(CPACK_PACKAGE_VENDOR "Stanford University, National Center of Biomedical Computation")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ReadMe.txt")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/Copyright.txt")
SET(CPACK_PACKAGE_VERSION_MAJOR "2")
SET(CPACK_PACKAGE_VERSION_MINOR "3")
SET(CPACK_PACKAGE_VERSION_PATCH "0")
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "OpenSim ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}")
IF(WIN32 AND NOT UNIX)
   # There is a bug in NSI that does not handle full unix paths properly. Make
   # sure there is at least one set of four (4) backlasshes.
   SET(CPACK_PACKAGE_ICON "${CMAKE_CURRENT_SOURCE_DIR}\\\\Install\\\\OpenSimInstallerIcon.bmp")
   # SET(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\OpenSim.exe")
   SET(CPACK_NSIS_DISPLAY_NAME  "${CPACK_PACKAGE_INSTALL_DIRECTORY} ")
   SET(CPACK_CREATE_DESKTOP_LINKS "${CPACK_PACKAGE_INSTALL_DIRECTORY}\\\\start_opensim.bat")
   SET(CPACK_NSIS_HELP_LINK "http:\\\\\\\\www.simtk.org/home/opensim")
   SET(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\www.simtk.org/home/opensim")
   SET(CPACK_NSIS_CONTACT "aymanh@simtk.org")
   SET(CPACK_NSIS_MODIFY_PATH ON)
   SET (CPACK_NSIS_EXTRA_INSTALL_COMMANDS
     "ExecWait '\\\"$INSTDIR\\\\bin\\\\vcredist_x86.exe\\\"'"
   )
ELSE(WIN32 AND NOT UNIX)
   SET(CPACK_STRIP_FILES "bin/MyExecutable")
   SET(CPACK_SOURCE_STRIP_FILES "")
ENDIF(WIN32 AND NOT UNIX)
SET(CPACK_PACKAGE_EXECUTABLES "OpenSim" "OpenSim")

INCLUDE (CPack)

ENDIF(OPENSIM_INSTALL_PACKAGE_BUILD)

#
# Build the documentation
#

#-----------------------------------------------------------------------------

SUBDIRS(Vendors OpenSim Install API Documentation)
