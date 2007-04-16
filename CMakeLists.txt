PROJECT (OpenSim)

SET(OPENSIM_MINIMAL_BUILD ON CACHE BOOL "Minimal build (only build essential libraries and excutables)")

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
	SET(SIMTK_DIR "c:/Program Files/SimTK/" CACHE STRING "Top-level directory of SimTK install")
ELSE(WIN32)
	SET(SIMTK_OPENSIM_INSTALL_DIR "/usr/local/SimTK/OpenSim")
	SET(SIMTK_DIR "/usr/local/SimTK" CACHE STRING "Top-level directory of SimTK install")
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

INCLUDE (Dart)

#
# Build the documentation
#

#-----------------------------------------------------------------------------
SUBDIRS(Vendors OpenSim)
