#-----------------------------------------------------------------------------
#set(CMAKE_DEBUG_POSTFIX "_d" CACHE INTERNAL "" FORCE)

subdirs(OpenSimJNI)

if(BUILD_JAVA_WRAPPING) 

## Now build jar file from the java files created by SWIG. This depends on finding an installation of Java
find_package( Java )
if( JAVA_COMPILE )
  # message( STATUS " ===========================================" )
  # message( STATUS "  Java" )
  # message( STATUS " ===========================================" )
  ## 

  get_filename_component(JAVA_BIN_PATH "${JAVA_COMPILE}" PATH)
  find_program( JAVA_JAVA_H javah PATHS ${JAVA_BIN_PATH} )

  # message( "JAVA_COMPILE=" ${JAVA_COMPILE} )
  # message( "JAVA_SRC=" ${CMAKE_BINARY_DIR}"/OpenSim/Wrapping/Java/OpenSimJNI/src/org/opensim/modeling/*.java" )
  # message( "WD=" ${CMAKE_BINARY_DIR}"/OpenSim/Wrapping/Java/OpenSimJNI/src/" )

  add_custom_target( JavaCompile
     COMMAND ${JAVA_COMPILE} 
     ${CMAKE_BINARY_DIR}/OpenSim/Wrapping/Java/OpenSimJNI/src/org/opensim/modeling/*.java 
     -source 1.6 -target 1.6
     WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/OpenSim/Wrapping/Java/OpenSimJNI/src/)
     
  add_custom_command(TARGET JavaCompile
                   POST_BUILD
                   COMMAND ${JAVA_ARCHIVE} -cvf org-opensim-modeling.jar
                   org/opensim/modeling/*.class
                   WORKING_DIRECTORY
                   ${CMAKE_BINARY_DIR}/OpenSim/Wrapping/Java/OpenSimJNI/src/
   )
endif( JAVA_COMPILE )
endif(BUILD_JAVA_WRAPPING)

install(DIRECTORY Matlab DESTINATION Scripts PATTERN ".svn" EXCLUDE 
                                             PATTERN "Matlab/Dynamic_Walker_Example" EXCLUDE)

