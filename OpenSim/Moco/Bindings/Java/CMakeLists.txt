
set(SWIG_JAVA_MOCO_INTERFACE_FILE 
    "../../../../Bindings/Java/swig/java_moco.i")
set(SWIG_JAVA_MOCO_CXX_FILE 
    "${CMAKE_CURRENT_BINARY_DIR}/java_moco.cxx")
set(SWIG_JAVA_MOCO_H_FILE   
    "${CMAKE_CURRENT_BINARY_DIR}/java_moco.h")

set(OPENSIM_JAVA_WRAPPING_PACKAGE "org.opensim.modeling")

# Replace periods with slashes (to make a path).
# Syntax for find-replace:
# string(REGEX REPLACE <pattern> <replacement string>
#                      <target variable> <source string>)
string(REGEX REPLACE "\\." "/"
       SWIG_JAVA_PACKAGE_PATH ${OPENSIM_JAVA_WRAPPING_PACKAGE})
string(REGEX REPLACE "\\." "-"
       SWIG_JAVA_JAR_NAME ${OPENSIM_JAVA_WRAPPING_PACKAGE})
set(SWIG_JAVA_JAR_NAME "${SWIG_JAVA_JAR_NAME}.jar")

set(SWIG_JAVA_SOURCE_BUILD_OUTPUT_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/src/${SWIG_JAVA_PACKAGE_PATH})
set(SWIG_JAVA_JAR_BUILD_OUTPUT_PATH
    "${CMAKE_CURRENT_BINARY_DIR}/src/${SWIG_JAVA_JAR_NAME}")

# We place the .java files in this folder.
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory
    ${SWIG_JAVA_SOURCE_BUILD_OUTPUT_DIR})


# Generate C++ code with SWIG.
# ----------------------------
# For building C++ wrapper.
find_package(JNI 1.7 REQUIRED)

# Avoid excessive compiler warnings. We set these COMPILE_OPTIONS in the root
# CMakeLists.txt.
set_directory_properties(PROPERTIES COMPILE_OPTIONS "")

function(MocoGenerateJavaWrapper
        NAME INPUT_INTERFACE_FILE OUTPUT_CXX_FILE OUTPUT_H_FILE)

    set(_swig_common_args -c++ -java
            -package ${OPENSIM_JAVA_WRAPPING_PACKAGE}
            -I${CMAKE_SOURCE_DIR}/Moco
            -I${CMAKE_SOURCE_DIR}/opensim-core
            -I${CMAKE_SOURCE_DIR}/opensim-core/Bindings
            -I${CMAKE_SOURCE_DIR}/opensim-core/Bindings/Java/swig
            -I${Simbody_INCLUDE_DIR}
            ${SWIG_FLAGS}
            ${INPUT_INTERFACE_FILE}
            )

    OpenSimFindSwigFileDependencies(_${NAME}_dependencies ${NAME}
        "${_swig_common_args}"
        )

    # This directory will hold the SWIG-generated .java files, but SWIG won't
    # create this directory for us.
    execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory
        ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME})
    add_custom_command(
        # This target actually creates a lot more (all the produced .java files)
        # but we will just use these two files as a proxy for all of those.
        OUTPUT ${OUTPUT_CXX_FILE} ${OUTPUT_H_FILE}
        # To avoid compiling stale .java files, delete any existing .java files.
        COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}/*
        COMMAND ${SWIG_EXECUTABLE}
            -v # verbose
            -o ${OUTPUT_CXX_FILE}
            -outdir ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}
            ${_swig_common_args}
        DEPENDS ${_${NAME}_dependencies}
        COMMENT "Generating Java bindings source code with SWIG: ${NAME}")
endfunction() 

MocoGenerateJavaWrapper(moco
    ${SWIG_JAVA_MOCO_INTERFACE_FILE}
    ${SWIG_JAVA_MOCO_CXX_FILE}
    ${SWIG_JAVA_MOCO_H_FILE})


# Compile the C++ wrapper into a library.
# ---------------------------------------
add_library(osimMocoJavaJNI
    SHARED 
    ${SWIG_JAVA_MOCO_CXX_FILE} 
    ${SWIG_JAVA_MOCO_H_FILE}
    )

# SYSTEM to avoid warnings.
target_include_directories(osimMocoJavaJNI SYSTEM PRIVATE
                           "${JAVA_INCLUDE_PATH}" "${JAVA_INCLUDE_PATH2}"
                           # For Bindings/OpenSimHeaders_common.h, etc.:
                           "${CMAKE_SOURCE_DIR}/opensim-core")

target_link_libraries(osimMocoJavaJNI osimMoco)

set_target_properties(osimMocoJavaJNI PROPERTIES
   PROJECT_LABEL "Java - osimMocoJavaJNI"
   FOLDER "Moco/Bindings"
)

# Compiler flags.
if(${CMAKE_CXX_COMPILER_ID} MATCHES "GNU" OR
        ${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    # We purposefully wrap deprecated functions, so no need to see such warnings.
    set(COMPILE_FLAGS "-Wno-deprecated-declarations")
elseif(${CMAKE_CXX_COMPILER_ID} MATCHES "MSVC")
    # Don't warn about:
    # 4996: deprecated functions.
    # 4114: "const const T"
    set(COMPILE_FLAGS "/wd4996 /wd4114")
endif()
set_source_files_properties(
        ${SWIG_JAVA_MOCO_CXX_FILE}
        PROPERTIES COMPILE_FLAGS "${COMPILE_FLAGS}")

MocoAddInstallRPATHSelf(TARGET osimMocoJavaJNI LOADER)
MocoAddInstallRPATHSimbody(TARGET osimMocoJavaJNI LOADER
        FROM "${CMAKE_INSTALL_LIBDIR}")

install(TARGETS osimMocoJavaJNI EXPORT MocoTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})


# Compile java sources.
# ---------------------
find_package(Java 1.7 REQUIRED)

# To avoid compiling stale .java files, delete any existing files and copy
# over the new files from the module-specific directories (see
# OpenSimJNI/CMakeLists.txt).
add_custom_command(
    OUTPUT "${SWIG_JAVA_JAR_BUILD_OUTPUT_PATH}"
    DEPENDS ${SWIG_JAVA_MOCO_CXX_FILE}
    COMMAND ${CMAKE_COMMAND} -E remove ${SWIG_JAVA_SOURCE_BUILD_OUTPUT_DIR}/*
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${OpenSim_JAVA_FILES_DIR}/opensim/modeling
            ${SWIG_JAVA_SOURCE_BUILD_OUTPUT_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_BINARY_DIR}/src/moco
            ${SWIG_JAVA_SOURCE_BUILD_OUTPUT_DIR}
    COMMAND ${JAVA_COMPILE} 
            org/opensim/modeling/*.java 
            -source 1.6 -target 1.6
    COMMAND ${JAVA_ARCHIVE} -cvf ${SWIG_JAVA_JAR_NAME}
            org/opensim/modeling/*.class
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/src
    COMMENT "Compiling Java sources and creating jar archive.")


# Umbrella target for assembling the java bindings in the build tree.
# -------------------------------------------------------------------
add_custom_target(MocoJavaBindings ALL
    DEPENDS "${SWIG_JAVA_JAR_BUILD_OUTPUT_PATH}"
    )

add_dependencies(MocoJavaBindings osimMocoJavaJNI)

set_target_properties(MocoJavaBindings PROPERTIES
    PROJECT_LABEL "Java - umbrella target"
    FOLDER "Moco/Bindings")


# Install.
# --------
install(FILES "${SWIG_JAVA_JAR_BUILD_OUTPUT_PATH}"
        DESTINATION ${MOCO_INSTALL_JAVAJARDIR})


# Java tests.
# -----------
if(BUILD_TESTING)
    add_subdirectory(Tests)
endif()


# Matlab.
# -------
add_subdirectory(Matlab)

