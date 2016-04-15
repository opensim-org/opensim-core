set(KIT JavaJNI)
set(UKIT JAVAJNI)

# For building C++ wrapper.
find_package(JNI 1.6 REQUIRED)

function(OpenSimGenerateJavaWrapper
        NAME INPUT_INTERFACE_FILE OUTPUT_CXX_FILE OUTPUT_H_FILE)
    add_custom_command(
        # This target actually creates a lot more (all the produced .java files)
        # but we will just use these two files as a proxy for all of those.
        OUTPUT ${OUTPUT_CXX_FILE} ${OUTPUT_H_FILE}
        COMMAND ${SWIG_EXECUTABLE} -v -c++ -java
                -package ${OPENSIM_JAVA_WRAPPING_PACKAGE}
                -I${OpenSim_SOURCE_DIR}
                -I${OpenSim_SOURCE_DIR}/Bindings
                -I${Simbody_INCLUDE_DIR}
                ${SWIG_FLAGS}
                -o ${OUTPUT_CXX_FILE}
                -outdir ${SWIG_JAVA_SOURCE_BUILD_OUTPUT_DIR}
                ${INPUT_INTERFACE_FILE}
        DEPENDS ${INPUT_INTERFACE_FILE}
        "${CMAKE_SOURCE_DIR}/Bindings/preliminaries.i"
        "${CMAKE_SOURCE_DIR}/Bindings/simbody.i"
        "${CMAKE_SOURCE_DIR}/Bindings/common.i"
        "${CMAKE_SOURCE_DIR}/Bindings/simulation.i"
        "${CMAKE_SOURCE_DIR}/Bindings/actuators.i"
        "${CMAKE_SOURCE_DIR}/Bindings/analyses.i"
        "${CMAKE_SOURCE_DIR}/Bindings/tools.i"
        "${CMAKE_SOURCE_DIR}/Bindings/OpenSimHeaders_common.h"
        "${CMAKE_SOURCE_DIR}/Bindings/OpenSimHeaders_simulation.h"
        "${CMAKE_SOURCE_DIR}/Bindings/OpenSimHeaders_actuators.h"
        "${CMAKE_SOURCE_DIR}/Bindings/OpenSimHeaders_analyses.h"
        "${CMAKE_SOURCE_DIR}/Bindings/OpenSimHeaders_tools.h"
        COMMENT "Generating Java bindings source code with SWIG: ${NAME}")
endfunction()

OpenSimGenerateJavaWrapper(simbody
    ${SWIG_JAVA_SIMBODY_INTERFACE_FILE}
    ${SWIG_JAVA_SIMBODY_CXX_FILE}
    ${SWIG_JAVA_SIMBODY_H_FILE})

OpenSimGenerateJavaWrapper(common
    ${SWIG_JAVA_COMMON_INTERFACE_FILE}
    ${SWIG_JAVA_COMMON_CXX_FILE}
    ${SWIG_JAVA_COMMON_H_FILE})

OpenSimGenerateJavaWrapper(simulation
    ${SWIG_JAVA_SIMULATION_INTERFACE_FILE}
    ${SWIG_JAVA_SIMULATION_CXX_FILE}
    ${SWIG_JAVA_SIMULATION_H_FILE})

OpenSimGenerateJavaWrapper(actuators-analysis-tools
    ${SWIG_JAVA_ACTUATORS_INTERFACE_FILE}
    ${SWIG_JAVA_ACTUATORS_CXX_FILE}
    ${SWIG_JAVA_ACTUATORS_H_FILE})

include_directories(${OpenSim_SOURCE_DIR} 
                    ${OpenSim_SOURCE_DIR}/Vendors 
                    ${JAVA_INCLUDE_PATH}
                    ${JAVA_INCLUDE_PATH2})

set(EXPORT_MACRO OSIM${UKIT}_EXPORTS)

add_library(osim${KIT} 
    SHARED 
    ${SWIG_JAVA_SIMBODY_CXX_FILE} 
    ${SWIG_JAVA_SIMBODY_H_FILE}
    ${SWIG_JAVA_COMMON_CXX_FILE} 
    ${SWIG_JAVA_COMMON_H_FILE}
    ${SWIG_JAVA_SIMULATION_CXX_FILE} 
    ${SWIG_JAVA_SIMULATION_H_FILE}
    ${SWIG_JAVA_ACTUATORS_CXX_FILE} 
    ${SWIG_JAVA_ACTUATORS_H_FILE} 
    OpenSimContext.cpp
    OpenSimContext.h)

target_link_libraries(osim${KIT} osimTools osimSimmFileWriter)
    
set_target_properties(osim${KIT} PROPERTIES
   DEFINE_SYMBOL ${EXPORT_MACRO}
   PROJECT_LABEL "Java - osim${KIT}"
   FOLDER "Bindings"
)

if(${OPENSIM_USE_INSTALL_RPATH})
    set_target_properties(osim${KIT} PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_FULL_LIBDIR}"
        )
endif()

# Shared libraries are needed at runtime for applications, so we put them
# at the top level in OpenSim/bin/*.dll (Windows) or OpenSim/lib/*.so (Linux)
# or OpemSim/lib/*.dylib (Mac). Windows .lib files, and Linux/Mac .a static
# archives are only needed at link time so go in sdk/lib.
install(TARGETS osim${KIT}
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${OPENSIM_INSTALL_ARCHIVEDIR}")

install(DIRECTORY ${SWIG_JAVA_SOURCE_BUILD_OUTPUT_DIR}/
    DESTINATION "${OPENSIM_INSTALL_JAVASRCDIR}/${SWIG_JAVA_PACKAGE_PATH}")

if(BUILD_TESTING)
    subdirs(Test)
endif()
