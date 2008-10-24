SET(KIT Common)
SET(UKIT COMMON)

##
## Version stuff
##
SET(OPENSIM_COMMON_LIBRARY_NAME osim${KIT})
SET(OPENSIM_COMMON_MAJOR_VERSION 0)
SET(OPENSIM_COMMON_MINOR_VERSION 7)
SET(OPENSIM_COMMON_BUILD_VERSION 1)
SET(OPENSIM_COMMON_COPYRIGHT_YEARS "2005-6")
# comma separated list of authors
SET(OPENSIM_COMMON_AUTHORS  "Clay_Anderson-Ayman_Habib_and_Peter_Loan") 


ADD_DEFINITIONS(-DOPENSIM_COMMON_LIBRARY_NAME=${OPENSIM_COMMON_LIBRARY_NAME}
-DOPENSIM_COMMON_MAJOR_VERSION=${OPENSIM_COMMON_MAJOR_VERSION}
-DOPENSIM_COMMON_MINOR_VERSION=${OPENSIM_COMMON_MINOR_VERSION}
-DOPENSIM_COMMON_BUILD_VERSION=${OPENSIM_COMMON_BUILD_VERSION}
-DOPENSIM_COMMON_COPYRIGHT_YEARS="${OPENSIM_COMMON_COPYRIGHT_YEARS}"
-DOPENSIM_COMMON_AUTHORS="${OPENSIM_COMMON_AUTHORS}"
-DOPENSIM_COMMON_TYPE="Shared")


FILE(GLOB SOURCE_FILES *.cpp gcvspl.c)
FILE(GLOB INCLUDE_FILES *.h gcvspl.h)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR}/Vendors ${XERCES_INCLUDE_DIR})

SET(EXPORT_MACRO OSIM${UKIT}_EXPORTS)

LINK_LIBRARIES( debug xerces-c_2D optimized  xerces-c_2
					${SIMTK_COMMON_LIB} ${SIMTK_MATH_LIB})

ADD_LIBRARY(osim${KIT} SHARED ${SOURCE_FILES} ${INCLUDE_FILES}) 

#-----------------------------------------------------------------------------
# Copy Libraries that we depend on to the output path so that exes and libs find them
#

FILE(GLOB LIB_FILES ${SIMTK_DIR}/core/lib/*.dll  ${SIMTK_DIR}/core/lib/*.lib 
					${OpenSim_SOURCE_DIR}/Vendors/xerces-c-src_2_7_0/lib/Win32/${VCVERSION}/*.lib
					${OpenSim_SOURCE_DIR}/Vendors/xerces-c-src_2_7_0/lib/Win32/${VCVERSION}/*.dll
					)

FOREACH (libFile ${LIB_FILES})
GET_FILENAME_COMPONENT(localName ${libFile} NAME)
 ADD_CUSTOM_COMMAND(
    TARGET osim${KIT}
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${libFile}
    ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_CFG_INTDIR}/${localName})
ENDFOREACH (libFile) 
#-----------------------------------------------------------------------------

SET_TARGET_PROPERTIES(osim${KIT} PROPERTIES
   DEFINE_SYMBOL ${EXPORT_MACRO}
   PROJECT_LABEL "Libraries - osim${KIT}"
)

INSTALL_TARGETS(/sdk/lib/ RUNTIME_DIRECTORY /bin osim${KIT})
INSTALL_FILES(/sdk/include/OpenSim/Common .h ${INCLUDE_FILES})

#
# Testing
#

#IF (UNIX)
#  ADD_DEFINITIONS(-fprofile-arcs -ftest-coverage)
#  LINK_LIBRARIES(gcov)
#ENDIF (UNIX)

IF(WIN32)
ENABLE_TESTING()

IF (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
ELSE (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH .)
ENDIF (EXECUTABLE_OUTPUT_PATH)
#

SUBDIRS(Test)
ENDIF(WIN32)