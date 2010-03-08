PROJECT (OpenSim)

SET(ARCH64 OFF CACHE BOOL "ON for 64bit builds, OFF for 32bit builds")

cmake_minimum_required(VERSION 2.6)
if(COMMAND cmake_policy)
        cmake_policy(SET CMP0003 NEW)
        cmake_policy(SET CMP0005 NEW)
endif(COMMAND cmake_policy)

#
# determine if we are building on a 32 or 64 bit machine
#
IF( ${CMAKE_SIZEOF_VOID_P} EQUAL 8 )
  SET( LIB64 64 )
ELSE( ${CMAKE_SIZEOF_VOID_P} EQUAL 8 )
  SET( LIB64  )
ENDIF( ${CMAKE_SIZEOF_VOID_P} EQUAL 8 )

IF( APPLE )
   SET( CMAKE_CXX_FLAGS "-mmacosx-version-min=10.4 -m32" )
   SET( LIB64  )
ENDIF( APPLE )

SET(BUILD_UTILITIES ON CACHE BOOL "Build rarely used tools (primarily to handle Storages and Controls)")

SET(BUILD_JAVA_WRAPPING OFF CACHE BOOL "Build Java wrapping (needed if you're building the GUI and have SWIG and Java installed on your machine.)")

SET(NameSpace "OpenSim_" CACHE STRING "Prefix for simtk lib names, includes trailing '_'. Set to '' to use stock SimTK libraries.")
MARK_AS_ADVANCED(NameSpace)

# Uncomment to make the linux Makefiles print the compile/link commands they are executing
#SET(CMAKE_VERBOSE_MAKEFILE ON)

IF(WIN32)
ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
ELSE(WIN32)
# Turn on all warnings in linux
SET(CMAKE_CXX_FLAGS -Wall)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

# Linux needs -D_DEBUG explicitly added
IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
ADD_DEFINITIONS(-D_DEBUG)
ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")
ENDIF(WIN32)

##################################################
## Define ARCH to
##			Win32/VC${version}
##		or	Mac
##################################################
SET(SHARED_LIB_EXT ".so" CACHE STRING "Extension for shared libraries")

IF(WIN32)
	IF(CMAKE_GENERATOR MATCHES  "Visual Studio 8")
	   SET(VCVERSION VC8)
	   SET(ARCH "Win32/VC8")
	ENDIF(CMAKE_GENERATOR MATCHES  "Visual Studio 8")
	IF(CMAKE_GENERATOR MATCHES  "Visual Studio 9")
	  SET(VCVERSION VC9)
	  SET(ARCH "Win32/VC9")
	ENDIF(CMAKE_GENERATOR MATCHES  "Visual Studio 9")
	SET(SHARED_LIB_EXT ".dll")
ELSE(WIN32)
	SET(NameSpace "")	## No renamed SimTK libraries except on Windows
	IF(APPLE)
	  SET(ARCH "Mac")
	  SET(SHARED_LIB_EXT ".dylib")
	ENDIF(APPLE)
ENDIF(WIN32)


# Build using the simtk headers and libraries checked in the repository. 
# Currently this only works on windows VC8 since these are the only libraries checked in
IF(WIN32)
    IF(NOT SimTK_SDK)
        SET(SimTK_SDK "C:/Program Files/SimTK")
    ENDIF(NOT SimTK_SDK)
ELSE(WIN32)
    IF(NOT SimTK_SDK)
        SET(SimTK_SDK "/usr/local/SimTK")
    ENDIF(NOT SimTK_SDK)
ENDIF(WIN32)

SET(USE_LIBRARIES_FROM_REPOSITORY ON CACHE BOOL "Use stock SimTK libraries based on SimtkCore 1.6")

IF(WIN32)
	SET(CMAKE_INSTALL_PREFIX "c:/Program Files/OpenSimTK" CACHE PATH "Install location to place headers and libraries")
	SET(SIMTK_DIR "c:/Program Files/SimTK/" CACHE PATH "Top-level directory of SimTK install")
ELSE(WIN32)
	SET(CMAKE_INSTALL_PREFIX "/usr/local/OpenSim")
	SET(SIMTK_DIR "/usr/local/SimTK" CACHE PATH "Top-level directory of SimTK install")
ENDIF(WIN32)

# If using prebuilt libraries reset SIMTK_DIR to relative path in source tree
IF(USE_LIBRARIES_FROM_REPOSITORY)
	SET(SIMTK_INCLUDE_DIR ${OpenSim_SOURCE_DIR}/Vendors/Include)
	SET(SIMTK_LIB_DIR ${OpenSim_SOURCE_DIR}/Vendors/lib/${ARCH})
ELSE(USE_LIBRARIES_FROM_REPOSITORY)
	SET(SIMTK_INCLUDE_DIR ${SIMTK_DIR}/Include)
	SET(SIMTK_LIB_DIR ${SIMTK_DIR}/lib)	
ENDIF(USE_LIBRARIES_FROM_REPOSITORY)

INSTALL(DIRECTORY ${SIMTK_INCLUDE_DIR} DESTINATION sdk/include/SimTK PATTERN ".svn" EXCLUDE)

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

SET(XERCES_DIR_NAME xerces-c_2_8_0)
SET(XERCES_INCLUDE_DIR ${OpenSim_SOURCE_DIR}/Vendors/${XERCES_DIR_NAME}/include)

INCLUDE_DIRECTORIES(${SIMTK_INCLUDE_DIR} ${XERCES_INCLUDE_DIR})

IF(WIN32)
	LINK_DIRECTORIES(${SIMTK_LIB_DIR} ${OpenSim_LIBRARY_DIR} ${OpenSim_SOURCE_DIR}/Vendors/${XERCES_DIR_NAME}/lib/${ARCH})
ELSE(WIN32)
	IF(APPLE)
		LINK_DIRECTORIES(${SIMTK_LIB_DIR} ${OpenSim_LIBRARY_DIR} ${OpenSim_SOURCE_DIR}/Vendors/${XERCES_DIR_NAME}/lib/${ARCH})
	ELSE(APPLE) # Linux?
		LINK_DIRECTORIES(${SIMTK_DIR}/lib${LIB64} ${OpenSim_LIBRARY_DIR} ${OpenSim_SOURCE_DIR}/Vendors/${XERCES_DIR_NAME}/lib)
	ENDIF(APPLE)
ENDIF(WIN32)

SET(USE_STATIC_SIMTK_LIBRARIES OFF CACHE BOOL "Link to static SimTK libraries (useful for debugging)")
IF(USE_STATIC_SIMTK_LIBRARIES)
	ADD_DEFINITIONS(-DSimTK_USE_STATIC_LIBRARIES)
	SET(SIMTK_COMMON_LIB debug ${NameSpace}SimTKcommon_static_d optimized ${NameSpace}SimTKcommon_static)
	SET(SIMTK_MATH_LIB debug ${NameSpace}SimTKmath_static_d optimized ${NameSpace}SimTKmath_static)
	SET(SIMTK_CPODES_LIB debug ${NameSpace}SimTKcpodes_static_d optimized ${NameSpace}SimTKcpodes_static)
	SET(SIMTK_SIMBODY_LIB debug ${NameSpace}SimTKsimbody_static_d optimized ${NameSpace}SimTKsimbody_static)
ELSE(USE_STATIC_SIMTK_LIBRARIES)
	SET(SIMTK_COMMON_LIB debug ${NameSpace}SimTKcommon_d optimized ${NameSpace}SimTKcommon)
	SET(SIMTK_MATH_LIB debug ${NameSpace}SimTKmath_d optimized ${NameSpace}SimTKmath)
	SET(SIMTK_CPODES_LIB debug ${NameSpace}SimTKcpodes_d optimized ${NameSpace}SimTKcpodes)
	SET(SIMTK_SIMBODY_LIB debug ${NameSpace}SimTKsimbody_d optimized ${NameSpace}SimTKsimbody)
ENDIF(USE_STATIC_SIMTK_LIBRARIES)

IF(WIN32)
SET(SIMTK_ALL_LIBS	${SIMTK_COMMON_LIB} 
					${SIMTK_MATH_LIB} 
					${SIMTK_CPODES_LIB} 
					${SIMTK_SIMBODY_LIB} 
					SimTKlapack 
					pthreadVC2)
SET(THIRDPARTY_DYNLIB_FILES	${SIMTK_LIB_DIR}/${NameSpace}SimTKcommon.dll 
						${SIMTK_LIB_DIR}/${NameSpace}SimTKmath.dll 
						${SIMTK_LIB_DIR}/${NameSpace}SimTKcpodes.dll 
						${SIMTK_LIB_DIR}/${NameSpace}SimTKsimbody.dll 
						${SIMTK_LIB_DIR}/pthreadVC2.dll
						${SIMTK_LIB_DIR}/SimTKlapack.dll
						${OpenSim_SOURCE_DIR}/Vendors/${XERCES_DIR_NAME}/lib/${ARCH}/xerces-c_2_8.dll)
ELSE(WIN32)
SET(SIMTK_ALL_LIBS	${SIMTK_COMMON_LIB} 
					${SIMTK_MATH_LIB} 
					${SIMTK_CPODES_LIB} 
					${SIMTK_SIMBODY_LIB} 
					SimTKlapack 
					pthread)
SET(THIRDPARTY_DYNLIB_FILES	${SIMTK_LIB_DIR}/lib${NameSpace}SimTKcommon${SHARED_LIB_EXT} 
						${SIMTK_LIB_DIR}/lib${NameSpace}SimTKmath${SHARED_LIB_EXT} 
						${SIMTK_LIB_DIR}/lib${NameSpace}SimTKcpodes${SHARED_LIB_EXT}
						${SIMTK_LIB_DIR}/lib${NameSpace}SimTKsimbody${SHARED_LIB_EXT}
						${SIMTK_LIB_DIR}/libpthread${SHARED_LIB_EXT}
						${SIMTK_LIB_DIR}/libSimTKlapack${SHARED_LIB_EXT}
						${OpenSim_SOURCE_DIR}/Vendors/${XERCES_DIR_NAME}/lib/${ARCH}/libxerces-c${SHARED_LIB_EXT})					
ENDIF(WIN32)

# We don't really want to copy the version of lapack we're using because that may be specific to our machine, so we take the generic lapack version (i.e. generic pentium 32 bit single proc)
IF(WIN32)
INSTALL_FILES(/bin/ FILES ${SIMTK_LIB_DIR}/${NameSpace}SimTKcommon.dll 
						${SIMTK_LIB_DIR}/${NameSpace}SimTKmath.dll 
						${SIMTK_LIB_DIR}/${NameSpace}SimTKcpodes.dll 
						${SIMTK_LIB_DIR}/${NameSpace}SimTKsimbody.dll 
						${SIMTK_LIB_DIR}/pthreadVC2.dll
						${SIMTK_LIB_DIR}/SimTKlapack.dll
						${SIMTK_LIB_DIR}/vcredist_x86.exe
						)
INSTALL_FILES(/sdk/lib/ FILES ${SIMTK_LIB_DIR}/${NameSpace}SimTKcommon.lib 
						${SIMTK_LIB_DIR}/${NameSpace}SimTKmath.lib 
						${SIMTK_LIB_DIR}/${NameSpace}SimTKcpodes.lib 
						${SIMTK_LIB_DIR}/${NameSpace}SimTKsimbody.lib 
						${SIMTK_LIB_DIR}/pthreadVC2.lib
						${SIMTK_LIB_DIR}/SimTKlapack.lib)

IF(OPENSIM_INSTALL_PACKAGE_BUILD)
# Install windows runtime libraries to support prebuilt libraries
INSTALL_FILES(/bin/ FILES 
					${OpenSim_SOURCE_DIR}/Vendors/vtk_dll/msvcr71.dll
		            ${OpenSim_SOURCE_DIR}/Vendors/vtk_dll/msvcp71.dll
		            ${SIMTK_LIB_DIR}/msvcr80.dll
		            ${SIMTK_LIB_DIR}/msvcp80.dll)
ENDIF(OPENSIM_INSTALL_PACKAGE_BUILD)

ELSE(WIN32)
		INSTALL_FILES(/sdk/lib/ FILES ${SIMTK_DIR}/lib${LIB64}/lib${NameSpace}SimTKcommon${SHARED_LIB_EXT} 
									${SIMTK_DIR}/lib${LIB64}/lib${NameSpace}SimTKmath${SHARED_LIB_EXT} 
									${SIMTK_DIR}/lib${LIB64}/libSimTKlapack${SHARED_LIB_EXT}
									${SIMTK_DIR}/lib${LIB64}/libSimTKcpodes${SHARED_LIB_EXT}
									${SIMTK_DIR}/lib${LIB64}/libSimTKsimbody${SHARED_LIB_EXT}
									${OpenSim_SOURCE_DIR}/Vendors/${XERCES_DIR_NAME}/lib/${ARCH}/libxerces-c${SHARED_LIB_EXT})
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
SET(CPACK_PACKAGE_VENDOR "Stanford University, National Center of BioComputation")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ReadMe.txt")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/Copyright.txt")
SET(CPACK_PACKAGE_VERSION_MAJOR "2")
SET(CPACK_PACKAGE_VERSION_MINOR "0")
SET(CPACK_PACKAGE_VERSION_PATCH "1")
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
