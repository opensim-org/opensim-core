#-----------------------------------------------------------------------------
#SET(CMAKE_DEBUG_POSTFIX "_d" CACHE INTERNAL "" FORCE)

SUBDIRS(OpenSimJNI)

IF(BUILD_JAVA_WRAPPING)	

## Now build jar file from the java files created by SWIG. This depends on finding an installation of Java
FIND_PACKAGE ( Java )
IF ( JAVA_COMPILE )
  # MESSAGE ( STATUS " ===========================================" )
  # MESSAGE ( STATUS "  Java" )
  # MESSAGE ( STATUS " ===========================================" )
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
  
  FILE(GLOB JAR_SRCS RELATIVE ${OpenSim_SOURCE_DIR}/Gui/opensim/modeling/src/org/opensim/modeling/ ${OpenSim_SOURCE_DIR}/Gui/opensim/modeling/src/org/opensim/modeling/*.java)
	ADD_CUSTOM_TARGET ( Javajar.classes
     COMMAND ${JAVA_COMPILE} -d ${OpenSim_SOURCE_DIR}/OpenSim/Java/build
     ${JAR_SRCS} 
     WORKING_DIRECTORY ${OpenSim_SOURCE_DIR}/Gui/opensim/modeling/src/org/opensim/modeling/)
     
  ADD_CUSTOM_COMMAND(TARGET Javajar.classes
                   POST_BUILD
                   COMMAND ${JAVA_ARCHIVE} cvf ${OpenSim_SOURCE_DIR}/OpenSim/Java/opensim.jar ${OpenSim_SOURCE_DIR}/OpenSim/Java/src/org
  )
ENDIF ( JAVA_COMPILE )
ENDIF(BUILD_JAVA_WRAPPING)

INSTALL(DIRECTORY Matlab DESTINATION Scripts PATTERN ".svn" EXCLUDE)

