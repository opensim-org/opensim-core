PROJECT (OpenSim)

SET(ARCH64 OFF CACHE BOOL "ON for 64bit builds, OFF for 32bit builds")

cmake_minimum_required(VERSION 2.4.6)				# ADDED BY BRIAN GARNER
if(COMMAND cmake_policy)
        cmake_policy(SET CMP0003 NEW)
        cmake_policy(SET CMP0005 NEW)
endif(COMMAND cmake_policy)


# Use /lib64 instead of /lib for 64 bit linux builds
IF(ARCH64)
SET(LIB64 64)
ELSE(ARCH64)
SET(LIB64)
ENDIF(ARCH64)

SET(OPENSIM_MINIMAL_BUILD OFF CACHE BOOL "Minimal build (only build essential libraries and excutables)")

# SET(INSTALL_HEADERS BUILD OFF CACHE BOOL "Copy required header files to install location")

# EXCLUDE_IF_MINIMAL_BUILD is a variable we set up which can be included in a SET_TARGET_PROPERTIES command for those
# targets that you only want built by default in non-minimal build settings
IF(OPENSIM_MINIMAL_BUILD)
SET(EXCLUDE_IF_MINIMAL_BUILD EXCLUDE_FROM_DEFAULT_BUILD on)
ELSE(OPENSIM_MINIMAL_BUILD)
SET(EXCLUDE_IF_MINIMAL_BUILD EXCLUDE_FROM_DEFAULT_BUILD off)
ENDIF(OPENSIM_MINIMAL_BUILD)

# Uncomment to make the linux Makefiles print the compile/link commands they are executing
#SET(CMAKE_VERBOSE_MAKEFILE ON)

IF(WIN32)
ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
ELSE(WIN32)
# Turn on all warnings in linux
SET(CMAKE_CXX_FLAGS -Wall)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

# Linux needs -D_DEBUG explicitly added
IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
ADD_DEFINITIONS(-D_DEBUG)
ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")
ENDIF(WIN32)

IF(WIN32)
	IF(CMAKE_GENERATOR MATCHES  "Visual Studio 8")
	   SET(VCVERSION VC8)
	ENDIF(CMAKE_GENERATOR MATCHES  "Visual Studio 8")
	IF(CMAKE_GENERATOR MATCHES  "Visual Studio 9")
	  SET(VCVERSION VC9)
	ENDIF(CMAKE_GENERATOR MATCHES  "Visual Studio 9")
ENDIF(WIN32)
# Build using the simtk headers and libraries checked in the repository. 
# Currently this only works on windows VC8 since these are the only libraries checked in
SET(USE_LIBRARIES_FROM_REPOSITORY ON CACHE BOOL "Use stock SimTK libraries 1.5.1 built with vc8")

IF(WIN32)
	SET(CMAKE_INSTALL_PREFIX "c:/Program Files/OpenSim_SimTK" CACHE PATH "Install location to place headers and libraries")
	SET(SIMTK_DIR "c:/Program Files/SimTK/" CACHE PATH "Top-level directory of SimTK install")
ELSE(WIN32)
	SET(CMAKE_INSTALL_PREFIX "/usr/local/OpenSim")
	SET(SIMTK_DIR "/usr/local/SimTK" CACHE PATH "Top-level directory of SimTK install")
ENDIF(WIN32)

# If using prebuilt libraries reset SIMTK_DIR to relative path in source tree
IF(USE_LIBRARIES_FROM_REPOSITORY)
	SET(SIMTK_DIR ${OpenSim_SOURCE_DIR}/Vendors/)
ENDIF(USE_LIBRARIES_FROM_REPOSITORY)
INSTALL(DIRECTORY ${SIMTK_DIR}/core/include DESTINATION sdk/include/SimTK PATTERN ".svn" EXCLUDE)

SET(OPENSIM_INSTALL_PACKAGE_BUILD OFF CACHE BOOL "Build an installer (to post on simtk.org)")
IF(OPENSIM_INSTALL_PACKAGE_BUILD)
IF(WIN32)
	SET(NETBEANS_INSTALL_DIR "C:/Program Files/netbeans-5.5/" CACHE PATH "Netbeans install directory")
	SET(VTK_LIB_DIR ${OpenSim_SOURCE_DIR}/Vendors/vtk_dll CACHE PATH "Prebuilt VTK-shared libraries")	
	SET(JRE_TO_PACKAGE "C:/Program Files/Java/jdk1.5.0_6/jre" CACHE PATH "JRE to package with OpenSim (jre dir)")
ENDIF(WIN32)
ENDIF(OPENSIM_INSTALL_PACKAGE_BUILD)

IF(NOT LIBRARY_OUTPUT_PATH)
  SET(LIBRARY_OUTPUT_PATH ${OpenSim_BINARY_DIR}/${VCVERSION} CACHE INTERNAL "Single output directory for building all libraries.")
ENDIF(NOT LIBRARY_OUTPUT_PATH)
IF(NOT EXECUTABLE_OUTPUT_PATH)
  SET(EXECUTABLE_OUTPUT_PATH ${OpenSim_BINARY_DIR}/${VCVERSION} CACHE INTERNAL "Single output directory for building all executables.")
ENDIF(NOT EXECUTABLE_OUTPUT_PATH)
SET(OpenSim_LIBRARY_DIR ${LIBRARY_OUTPUT_PATH})
SET(OpenSim_EXECUTABLE_DIR ${EXECUTABLE_OUTPUT_PATH})
SET(CMAKE_DEBUG_POSTFIX "_d" CACHE INTERNAL "" FORCE)

# SET(CMAKE_INSTALL_PREFIX ${SIMTK_OPENSIM_INSTALL_DIR})

SET(XERCES_DIR_NAME xerces-c-src_2_7_0)
SET(XERCES_INCLUDE_DIR ${OpenSim_SOURCE_DIR}/Vendors/${XERCES_DIR_NAME}/include)

INCLUDE_DIRECTORIES(${SIMTK_DIR}/core/include ${XERCES_INCLUDE_DIR})

IF(WIN32)
	LINK_DIRECTORIES(${SIMTK_DIR}/core/lib 
		${OpenSim_LIBRARY_DIR} 
		${OpenSim_SOURCE_DIR}/Vendors/${XERCES_DIR_NAME}/lib/Win32/${VCVERSION})
ELSE(WIN32)
	LINK_DIRECTORIES(${SIMTK_DIR}/core/lib${LIB64}  ${OpenSim_LIBRARY_DIR} ${OpenSim_SOURCE_DIR}/Vendors/${XERCES_DIR_NAME}/lib)
ENDIF(WIN32)

SET(USE_STATIC_SIMTK_LIBRARIES OFF CACHE BOOL "Link to static SimTK libraries (useful for debugging)")
IF(USE_STATIC_SIMTK_LIBRARIES)
	ADD_DEFINITIONS(-DSimTK_USE_STATIC_LIBRARIES)
	SET(SIMTK_COMMON_LIB debug SimTKcommon_static_d optimized SimTKcommon_static)
	SET(SIMTK_MATH_LIB debug SimTKmath_static_d optimized SimTKmath_static SimTKlapack)
ELSE(USE_STATIC_SIMTK_LIBRARIES)
	SET(SIMTK_COMMON_LIB debug SimTKcommon_d optimized SimTKcommon)
	SET(SIMTK_MATH_LIB debug SimTKmath_d optimized SimTKmath SimTKlapack)
ENDIF(USE_STATIC_SIMTK_LIBRARIES)


# We don't really want to copy the version of lapack we're using because that may be specific to our machine, so we take the generic lapack version (i.e. generic pentium 32 bit single proc)
IF(WIN32)
INSTALL_FILES(/bin/ FILES ${SIMTK_DIR}/core/lib/SimTKcommon.dll 
						${SIMTK_DIR}/core/lib/SimTKmath.dll 
						${SIMTK_DIR}/core/lib/SimTKcpodes.dll 
						${SIMTK_DIR}/core/lib/SimTKsimbody.dll 
						${SIMTK_DIR}/core/lib/pthreadVC2.dll
						${SIMTK_DIR}/core/lib/SimTKlapack.dll)
INSTALL_FILES(/sdk/lib/ FILES ${SIMTK_DIR}/core/lib/SimTKcommon.lib 
						${SIMTK_DIR}/core/lib/SimTKmath.lib 
						${SIMTK_DIR}/core/lib/SimTKcpodes.lib 
						${SIMTK_DIR}/core/lib/SimTKsimbody.lib 
						${SIMTK_DIR}/core/lib/pthreadVC2.lib
						${SIMTK_DIR}/core/lib/SimTKlapack.lib)

IF(OPENSIM_INSTALL_PACKAGE_BUILD)
# Install windows runtime libraries to support prebuilt libraries
INSTALL_FILES(/bin/ FILES 
					${OpenSim_SOURCE_DIR}/Vendors/vtk_dll/msvcr71.dll
		            ${OpenSim_SOURCE_DIR}/Vendors/vtk_dll/msvcp71.dll
		            ${SIMTK_DIR}/core/lib/msvcr80.dll
		            ${SIMTK_DIR}/core/lib/msvcp80.dll)
ENDIF(OPENSIM_INSTALL_PACKAGE_BUILD)

ELSE(WIN32)
INSTALL_FILES(/sdk/lib/ FILES ${SIMTK_DIR}/core/lib${LIB64}/libSimTKcommon.so ${SIMTK_DIR}/core/lib${LIB64}/libSimTKmath.so ${SIMTK_DIR}/core/lib${LIB64}/generic_lapack/libSimTKlapack.so)
ENDIF(WIN32)

## Install other files (batch file, doc, api)
INCLUDE (Dart)

IF(OPENSIM_INSTALL_PACKAGE_BUILD)
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "OpenSim project")
SET(CPACK_PACKAGE_VENDOR "Stanford University, National Center of BioComputation")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ReadMe.txt")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/Copyright.txt")
SET(CPACK_PACKAGE_VERSION_MAJOR "1")
SET(CPACK_PACKAGE_VERSION_MINOR "8")
SET(CPACK_PACKAGE_VERSION_PATCH "0")
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "OpenSim ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}")
IF(WIN32 AND NOT UNIX)
   # There is a bug in NSI that does not handle full unix paths properly. Make
   # sure there is at least one set of four (4) backlasshes.
   SET(CPACK_PACKAGE_ICON "${CMAKE_CURRENT_SOURCE_DIR}\\\\Install\\\\OpenSimInstallerIcon.bmp")
   # SET(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\OpenSim.exe")
   SET(CPACK_NSIS_DISPLAY_NAME  "${CPACK_PACKAGE_INSTALL_DIRECTORY} Application")
   SET(CPACK_NSIS_HELP_LINK "http:\\\\\\\\www.simtk.org/home/opensim")
   SET(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\www.simtk.org/home/opensim")
   SET(CPACK_NSIS_CONTACT "aymanh@simtk.org")
   SET(CPACK_NSIS_MODIFY_PATH ON)
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

SUBDIRS(Vendors OpenSim Documentation Install API)
