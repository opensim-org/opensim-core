PROJECT (OpenSim)

SET(OPENSIM_MINIMAL_BUILD ON CACHE BOOL "Minimal build (only build essential libraries and excutables)")

# EXCLUDE_IF_MINIMAL_BUILD is a variable we set up which can be included in a SET_TARGET_PROPERTIES command for those
# targets that you only want built by default in non-minimal build settings
IF(OPENSIM_MINIMAL_BUILD)
SET(EXCLUDE_IF_MINIMAL_BUILD EXCLUDE_FROM_DEFAULT_BUILD on)
ELSE(OPENSIM_MINIMAL_BUILD)
SET(EXCLUDE_IF_MINIMAL_BUILD)
ENDIF(OPENSIM_MINIMAL_BUILD)

# Uncomment to make the linux Makefiles print the compile/link commands they are executing
#SET(CMAKE_VERBOSE_MAKEFILE ON)

IF(NOT WIN32)
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
ENDIF(NOT WIN32)

SET(RDI_ARCH $ENV{RDI_ARCH})
IF(WIN32)
	IF(${RDI_ARCH} MATCHES ".+")
	ELSE(${RDI_ARCH} MATCHES ".+")
		SET(RDI_ARCH VC71)
	ENDIF(${RDI_ARCH} MATCHES ".+")
ENDIF(WIN32)

IF(WIN32)
	SET(SIMTK_OPENSIM_INSTALL_DIR "c:/Program Files/SimTK/OpenSim")
	SET(SIMTK_DIR "c:/Program Files/SimTK/" CACHE PATH "Top-level directory of SimTK install")
	SET(NETBEANS_INSTALL_DIR "C:/netbeans-5.5/" CACHE PATH "Netbeans install directory")
	SET(VTK_LIB_DIR "C:/VTK50/vtk-5.0.0/VTKBuild/bin" CACHE PATH "Prebuilt VTK-shared libraries")	
ELSE(WIN32)
	SET(SIMTK_OPENSIM_INSTALL_DIR "/usr/local/SimTK/OpenSim")
	SET(SIMTK_DIR "/usr/local/SimTK" CACHE PATH "Top-level directory of SimTK install")
ENDIF(WIN32)

IF(NOT LIBRARY_OUTPUT_PATH)
  SET(LIBRARY_OUTPUT_PATH ${OpenSim_BINARY_DIR}/${RDI_ARCH} CACHE INTERNAL "Single output directory for building all libraries.")
ENDIF(NOT LIBRARY_OUTPUT_PATH)
IF(NOT EXECUTABLE_OUTPUT_PATH)
  SET(EXECUTABLE_OUTPUT_PATH ${OpenSim_BINARY_DIR}/${RDI_ARCH} CACHE INTERNAL "Single output directory for building all executables.")
ENDIF(NOT EXECUTABLE_OUTPUT_PATH)
SET(OpenSim_LIBRARY_DIR ${LIBRARY_OUTPUT_PATH})
SET(OpenSim_EXECUTABLE_DIR ${EXECUTABLE_OUTPUT_PATH})
SET(CMAKE_DEBUG_POSTFIX "_d" CACHE INTERNAL "" FORCE)

SET(CMAKE_INSTALL_PREFIX ${SIMTK_OPENSIM_INSTALL_DIR})

IF(WIN32)
SET(XERCES_DIR_NAME xerces-c-src_2_7_0)
SET(XERCES_INCLUDE_DIR ${OpenSim_SOURCE_DIR}/Vendors/${XERCES_DIR_NAME}/src)
ELSE(WIN32)
SET(XERCES_INCLUDE_DIR "")
ENDIF(WIN32)

INCLUDE_DIRECTORIES(${SIMTK_DIR}/core/include)

IF(WIN32)
	LINK_DIRECTORIES(${SIMTK_DIR}/core/lib ${OpenSim_LIBRARY_DIR})
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
INSTALL_FILES(/bin/ FILES ${SIMTK_DIR}/core/lib/SimTKcommon.dll ${SIMTK_DIR}/core/lib/SimTKmath.dll ${SIMTK_DIR}/core/lib/generic_lapack/SimTKlapack.dll)
INSTALL_FILES(/lib/ FILES ${SIMTK_DIR}/core/lib/SimTKcommon.lib ${SIMTK_DIR}/core/lib/SimTKmath.lib ${SIMTK_DIR}/core/lib/generic_lapack/SimTKlapack.lib)
ELSE(WIN32)
INSTALL_FILES(/bin/ FILES ${SIMTK_DIR}/core/lib/libSimTKcommon.so ${SIMTK_DIR}/core/lib/libSimTKmath.so ${SIMTK_DIR}/core/lib/generic_lapack/libSimTKlapack.so)
ENDIF(WIN32)

INCLUDE (Dart)

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "OpenSim project")
SET(CPACK_PACKAGE_VENDOR "Stanford University, National Center of BioComputation")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ReadMe.txt")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/Copyright.txt")
SET(CPACK_PACKAGE_VERSION_MAJOR "0")
SET(CPACK_PACKAGE_VERSION_MINOR "9")
SET(CPACK_PACKAGE_VERSION_PATCH "0")
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "OpenSim ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}")
IF(WIN32 AND NOT UNIX)
   # There is a bug in NSI that does not handle full unix paths properly. Make
   # sure there is at least one set of four (4) backlasshes.
   # SET(CPACK_PACKAGE_ICON "${CMAKE_CURRENT_SOURCE_DIR}\\\\OpenSimIcon.bmp")
   # SET(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\OpenSim.exe")
   SET(CPACK_NSIS_DISPLAY_NAME  "${CPACK_PACKAGE_INSTALL_DIRECTORY}OpenSimPlatform")
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

#
# Build the documentation
#

#-----------------------------------------------------------------------------
SUBDIRS(Vendors OpenSim Documentation)
