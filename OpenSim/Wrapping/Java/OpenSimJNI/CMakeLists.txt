set(KIT JavaJNI)
set(UKIT JAVAJNI)

if(BUILD_JAVA_WRAPPING) 

include(${CMAKE_ROOT}/Modules/FindJava.cmake)
include(${CMAKE_ROOT}/Modules/FindJNI.cmake)

if(JAVA_INCLUDE_PATH)

# Run command to generate OpenSimJNI_wrap.cxx
set(swig_generated_file_fullname
    ${CMAKE_CURRENT_BINARY_DIR}/OpenSimJNI_wrap.cxx)
set(swig_generated_header_fullname
    ${CMAKE_CURRENT_BINARY_DIR}/OpenSimJNI_wrap.h)
set(swig_interface_file_fullname
    ${OpenSim_SOURCE_DIR}/OpenSim/Wrapping/Java/swig/javaWrapOpenSim.i)
set(OPENSIM_JAVA_WRAPPING_PACKAGE "org.opensim.modeling"
    CACHE STRING
    "The wrapping is built as the Java package specified in this variable.")

# Replace periods with slashes (to make a path).
# Syntax for find-replace:
# string(REGEX REPLACE <pattern> <replacement string>
#                      <target variable> <source string>)
string(REGEX REPLACE "\\." "/"
       SWIG_JAVA_PACKAGE_PATH ${OPENSIM_JAVA_WRAPPING_PACKAGE})

set(SWIG_JAVA_SOURCE_BUILD_OUTPUT_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/src/${SWIG_JAVA_PACKAGE_PATH})

# We place the .java files in this folder.
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory
    ${SWIG_JAVA_SOURCE_BUILD_OUTPUT_DIR})

add_custom_command(
    OUTPUT "${swig_generated_file_fullname}" "${swig_generated_header_fullname}"
    COMMAND ${SWIG_EXECUTABLE} -v -c++ -java
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
add_custom_target(JavaWrap
    DEPENDS "${swig_generated_file_fullname}"
            "${swig_generated_header_fullname}")

set_target_properties(JavaWrap PROPERTIES
    PROJECT_LABEL "Java Bindings - Generate")


set(SOURCE_FILES "${swig_generated_file_fullname}" OpenSimContext.cpp)
set(INCLUDE_FILES "${swig_generated_header_fullname}" OpenSimContext.h)

include_directories(${OpenSim_SOURCE_DIR} 
                    ${OpenSim_SOURCE_DIR}/Vendors 
                    ${JAVA_INCLUDE_PATH}
                    ${JAVA_INCLUDE_PATH2}
)

set(EXPORT_MACRO OSIM${UKIT}_EXPORTS)

link_libraries(osimCommon osimSimulation osimAnalyses osimTools
    osimSimmFileWriter)

add_library(osim${KIT} SHARED ${SOURCE_FILES} ${INCLUDE_FILES}) 

set_target_properties(osim${KIT} PROPERTIES
   DEFINE_SYMBOL ${EXPORT_MACRO}
   PROJECT_LABEL "Java Bindings - osim${KIT}"
)

add_dependencies(osim${KIT} JavaWrap)

# Shared libraries are needed at runtime for applications, so we put them
# at the top level in OpenSim/bin/*.dll (Windows) or OpenSim/lib/*.so (Linux)
# or OpemSim/lib/*.dylib (Mac). Windows .lib files, and Linux/Mac .a static
# archives are only needed at link time so go in sdk/lib.
if(WIN32)
    install(TARGETS osim${KIT}
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION sdk/lib
            ARCHIVE DESTINATION sdk/lib)
else()
    install(TARGETS osim${KIT}
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION sdk/lib)
endif()

install(DIRECTORY ${SWIG_JAVA_SOURCE_BUILD_OUTPUT_DIR}/
    DESTINATION sdk/Java/${SWIG_JAVA_PACKAGE_PATH})

if(BUILD_TESTING)
    subdirs(Test)
endif(BUILD_TESTING)


endif(JAVA_INCLUDE_PATH)
endif(BUILD_JAVA_WRAPPING)
