#-----------------------------------------------------------------------------
#SET(CMAKE_DEBUG_POSTFIX "_d" CACHE INTERNAL "" FORCE)

SUBDIRS(OpenSimJNI)

## Now build jar file from the java files created by SWIG. This depends on finding an installation of Java
FIND_PACKAGE ( Java )
IF ( JAVA_COMPILE )
  MESSAGE ( STATUS " ===========================================" )
  MESSAGE ( STATUS "  Java" )
  MESSAGE ( STATUS " ===========================================" )
  ## 
  EXECUTE_PROCESS ( COMMAND ${JAVA_COMPILE} -version
                    OUTPUT_VARIABLE JAVA_OUTPUT
                    ERROR_VARIABLE  JAVA_ERROR_OUTPUT
                    RESULT_VARIABLE JAVA_RETURN_VALUE )
  IF ( NOT JAVA_RETURN_VALUE )
    STRING ( REGEX MATCH [0-9._]+ JAVA_VERSION ${JAVA_ERROR_OUTPUT} )
  ENDIF ( NOT JAVA_RETURN_VALUE )

  GET_FILENAME_COMPONENT (JAVA_BIN_PATH "${JAVA_COMPILE}" PATH)
  FIND_PROGRAM ( JAVA_JAVA_H javah PATHS ${JAVA_BIN_PATH} )

  MESSAGE ( STATUS "  JAVA_VERSION:           " ${JAVA_VERSION} )
  MESSAGE ( STATUS "  JAVA_RUNTIME:           " ${JAVA_RUNTIME} )
  MESSAGE ( STATUS "  JAVA_COMPILE:           " ${JAVA_COMPILE} )
  MESSAGE ( STATUS "  JAVA_ARCHIVE:           " ${JAVA_ARCHIVE} )
  MESSAGE ( STATUS "  JAVA_JAVA_H:            " ${JAVA_JAVA_H} )
  MESSAGE ( STATUS "  JAVA_INCLUDE_PATH:      " ${JAVA_INCLUDE_PATH} )
  MESSAGE ( STATUS "  JAVA_INCLUDE_PATH2:     " ${JAVA_INCLUDE_PATH2} )
  MESSAGE ( STATUS "  JAVA_JVM_LIBRARY:       " ${JAVA_JVM_LIBRARY} )
  
  FILE(GLOB JAR_SRCS RELATIVE ${OpenSim_SOURCE_DIR}/OpenSim/Java/src/org/opensim/modeling/ ${OpenSim_SOURCE_DIR}/OpenSim/Java/src/org/opensim/modeling/*.java)
	ADD_CUSTOM_TARGET ( jar.classes
     COMMAND ${JAVA_COMPILE} -d ${OpenSim_SOURCE_DIR}/OpenSim/Java/build
     ${JAR_SRCS} 
     WORKING_DIRECTORY ${OpenSim_SOURCE_DIR}/OpenSim/Java/src/org/opensim/modeling/)
     
  
ENDIF ( JAVA_COMPILE )
