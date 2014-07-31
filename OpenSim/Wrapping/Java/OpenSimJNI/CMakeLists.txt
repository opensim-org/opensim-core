SET(KIT JavaJNI)
SET(UKIT JAVAJNI)

IF(BUILD_JAVA_WRAPPING)	

INCLUDE(${CMAKE_ROOT}/Modules/FindJava.cmake)
INCLUDE(${CMAKE_ROOT}/Modules/FindJNI.cmake)

IF(JAVA_INCLUDE_PATH)

# Run command to generate OpenSimJNI_wrap.cxx
SET(swig_generated_file_fullname
    ${CMAKE_CURRENT_BINARY_DIR}/OpenSimJNI_wrap.cxx)
SET(swig_generated_header_fullname
    ${CMAKE_CURRENT_BINARY_DIR}/OpenSimJNI_wrap.h)
SET(swig_interface_file_fullname
    ${OpenSim_SOURCE_DIR}/OpenSim/Wrapping/Java/swig/javaWrapOpenSim.i)
SET(OPENSIM_JAVA_WRAPPING_PACKAGE "org.opensim.modeling"
    CACHE STRING
    "The wrapping is built as the Java package specified in this variable.")

# Replace periods with slashes (to make a path).
# Syntax for find-replace:
# STRING(REGEX REPLACE <pattern> <replacement string>
#                      <target variable> <source string>)
STRING(REGEX REPLACE "\\." "/"
       SWIG_JAVA_PACKAGE_PATH ${OPENSIM_JAVA_WRAPPING_PACKAGE})

SET(SWIG_JAVA_SOURCE_BUILD_OUTPUT_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/src/${SWIG_JAVA_PACKAGE_PATH})

# We place the .java files in this folder.
EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E make_directory
    ${SWIG_JAVA_SOURCE_BUILD_OUTPUT_DIR})

SET(SimTK_INCLUDE_PATH ${SIMTK_INCLUDE_DIR})

ADD_CUSTOM_COMMAND(
    OUTPUT "${swig_generated_file_fullname}" "${swig_generated_header_fullname}"
    COMMAND ${SWIG_EXECUTABLE} -v -c++ -java
        # This command line option will remove the 
        # generation of superfluous virtual method wrappers.
        -fvirtual
        -package ${OPENSIM_JAVA_WRAPPING_PACKAGE}
        -I${OpenSim_SOURCE_DIR}
        -I${Simbody_INCLUDE_DIR}
        -I${OpenSim_SOURCE_DIR}/OpenSim/Wrapping
    	-o ${swig_generated_file_fullname}
        -outdir ${SWIG_JAVA_SOURCE_BUILD_OUTPUT_DIR}
        ${swig_interface_file_fullname}
    DEPENDS "${swig_interface_file_fullname}")
    
# This target actually depends on a lot more (all the produced .java files); 
# but we will just use these two files as a proxy for all of those. All we need
# is for this target to depend on outputs of the command above.
ADD_CUSTOM_TARGET(JavaWrap
    DEPENDS "${swig_generated_file_fullname}"
            "${swig_generated_header_fullname}")

SET_TARGET_PROPERTIES(JavaWrap PROPERTIES
    PROJECT_LABEL "Java Bindings - Generate")


SET(SOURCE_FILES "${swig_generated_file_fullname}" OpenSimContext.cpp)
SET(INCLUDE_FILES "${swig_generated_header_fullname}" OpenSimContext.h)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
					${OpenSim_SOURCE_DIR}/Vendors 
					${JAVA_INCLUDE_PATH}
					${JAVA_INCLUDE_PATH2}
)

SET(EXPORT_MACRO OSIM${UKIT}_EXPORTS)

LINK_LIBRARIES(osimCommon osimSimulation osimAnalyses osimTools
    osimSimmFileWriter)

ADD_LIBRARY(osim${KIT} SHARED ${SOURCE_FILES} ${INCLUDE_FILES}) 

SET_TARGET_PROPERTIES(osim${KIT} PROPERTIES
   DEFINE_SYMBOL ${EXPORT_MACRO}
   PROJECT_LABEL "Java Bindings - osim${KIT}"
)

ADD_DEPENDENCIES(osim${KIT} JavaWrap)

# Shared libraries are needed at runtime for applications, so we put them
# at the top level in OpenSim/bin/*.dll (Windows) or OpenSim/lib/*.so (Linux)
# or OpemSim/lib/*.dylib (Mac). Windows .lib files, and Linux/Mac .a static
# archives are only needed at link time so go in sdk/lib.
IF(WIN32)
    INSTALL(TARGETS osim${KIT}
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION sdk/lib
            ARCHIVE DESTINATION sdk/lib)
ELSE()
    INSTALL(TARGETS osim${KIT}
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION sdk/lib)
ENDIF()

INSTALL(DIRECTORY ${SWIG_JAVA_SOURCE_BUILD_OUTPUT_DIR}/
    DESTINATION sdk/Java/${SWIG_JAVA_PACKAGE_PATH})

IF(BUILD_TESTING)
	SUBDIRS(Test)
ENDIF(BUILD_TESTING)


ENDIF (JAVA_INCLUDE_PATH)
ENDIF(BUILD_JAVA_WRAPPING)
