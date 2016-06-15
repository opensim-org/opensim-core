set(KIT JavaJNI)
set(UKIT JAVAJNI)

# For building C++ wrapper.
find_package(JNI 1.6 REQUIRED)

function(OpenSimGenerateJavaWrapper
        NAME INPUT_INTERFACE_FILE OUTPUT_CXX_FILE OUTPUT_H_FILE)

    set(_swig_common_args -c++ -java
            -package ${OPENSIM_JAVA_WRAPPING_PACKAGE}
            -I${OpenSim_SOURCE_DIR}
            -I${OpenSim_SOURCE_DIR}/Bindings
            -I${Simbody_INCLUDE_DIR}
            ${SWIG_FLAGS}
            ${INPUT_INTERFACE_FILE}
            )

    OpenSimFindSwigFileDependencies(_${NAME}_dependencies ${NAME}
        "${_swig_common_args}"
        )

    add_custom_command(
        # This target actually creates a lot more (all the produced .java files)
        # but we will just use these two files as a proxy for all of those.
        OUTPUT ${OUTPUT_CXX_FILE} ${OUTPUT_H_FILE}
        COMMAND ${SWIG_EXECUTABLE}
            -v # verbose
            -o ${OUTPUT_CXX_FILE}
            -outdir ${SWIG_JAVA_SOURCE_BUILD_OUTPUT_DIR}
            ${_swig_common_args}
        DEPENDS ${_${NAME}_dependencies}
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

# Compiler flags.
# We purposefully wrap deprecated functions, so no need to see such warnings.
if(${CMAKE_CXX_COMPILER_ID} MATCHES "GNU" OR
        ${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    set(COMPILE_FLAGS "-Wno-deprecated-declarations")
elseif(${CMAKE_CXX_COMPILER_ID} MATCHES "MSVC")
    set(COMPILE_FLAGS "/wd4996")
endif()
set_source_files_properties(
        ${SWIG_JAVA_SIMBODY_CXX_FILE}
        ${SWIG_JAVA_COMMON_CXX_FILE}
        ${SWIG_JAVA_SIMULATION_CXX_FILE}
        ${SWIG_JAVA_ACTUATORS_CXX_FILE}
        PROPERTIES COMPILE_FLAGS "${COMPILE_FLAGS}")

if(${OPENSIM_USE_INSTALL_RPATH})
    set_target_properties(osim${KIT} PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
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

# Don't install .class files.
install(DIRECTORY ${SWIG_JAVA_SOURCE_BUILD_OUTPUT_DIR}/
    DESTINATION "${OPENSIM_INSTALL_JAVASRCDIR}/${SWIG_JAVA_PACKAGE_PATH}"
    PATTERN "*.class" EXCLUDE
    )

if(BUILD_TESTING)
    subdirs(Test)
endif()
