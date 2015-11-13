set(KIT JavaJNI)
set(UKIT JAVAJNI)

# For building C++ wrapper.
find_package(JNI 1.6 REQUIRED)

add_custom_command(
    # This target actually creates a lot more (all the produced .java files); 
    # but we will just use these two files as a proxy for all of those.
    OUTPUT "${swig_generated_file_fullname}" "${swig_generated_header_fullname}"
    COMMAND ${SWIG_EXECUTABLE} -v -c++ -java
        -package ${OPENSIM_JAVA_WRAPPING_PACKAGE}
        -I${OpenSim_SOURCE_DIR}
        -I${OpenSim_SOURCE_DIR}/Bindings
        -I${Simbody_INCLUDE_DIR}
        -o ${swig_generated_file_fullname}
        -outdir ${SWIG_JAVA_SOURCE_BUILD_OUTPUT_DIR}
        ${swig_interface_file_fullname}
    DEPENDS "${swig_interface_file_fullname}"
            "${CMAKE_SOURCE_DIR}/Bindings/preliminaries.i"
            "${CMAKE_SOURCE_DIR}/Bindings/simbody.i"
            "${CMAKE_SOURCE_DIR}/Bindings/opensim.i"
            "${CMAKE_SOURCE_DIR}/Bindings/OpenSimHeaders.h"
    COMMENT "Generating java bindings source code with SWIG."
    )

set(SOURCE_FILES "${swig_generated_file_fullname}" OpenSimContext.cpp)
set(INCLUDE_FILES "${swig_generated_header_fullname}" OpenSimContext.h)

include_directories(${OpenSim_SOURCE_DIR} 
                    ${OpenSim_SOURCE_DIR}/Vendors 
                    ${JAVA_INCLUDE_PATH}
                    ${JAVA_INCLUDE_PATH2}
)

set(EXPORT_MACRO OSIM${UKIT}_EXPORTS)

add_library(osim${KIT} SHARED ${SOURCE_FILES} ${INCLUDE_FILES}) 

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
