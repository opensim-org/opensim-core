
# Generate C++ and Java wrapper code with SWIG.
# ---------------------------------------------

set(swig_generated_file_fullname
    ${CMAKE_CURRENT_BINARY_DIR}/OpenSimJNI/OpenSimJNI_wrap.cxx)
set(swig_generated_header_fullname
    ${CMAKE_CURRENT_BINARY_DIR}/OpenSimJNI/OpenSimJNI_wrap.h)
set(swig_interface_file_fullname
    ${CMAKE_CURRENT_SOURCE_DIR}/swig/javaWrapOpenSim.i)
set(OPENSIM_JAVA_WRAPPING_PACKAGE "org.opensim.modeling"
    CACHE STRING
    "The wrapping is built as the Java package specified in this variable.")

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

# The actual SWIG command is in the OpenSimJNI folder, since the custom command
# must be defined in the same directory that uses it (for the osimJavaJNI
# target).

# Compile the C++ wrapper into a library.
# ---------------------------------------
add_subdirectory(OpenSimJNI)

# Compile java sources.
# ---------------------
find_package(Java 1.6 REQUIRED)

add_custom_command(
    OUTPUT "${SWIG_JAVA_JAR_BUILD_OUTPUT_PATH}"
    DEPENDS "${swig_generated_file_fullname}"
    COMMAND ${JAVA_COMPILE} 
            org/opensim/modeling/*.java 
            -source 1.6 -target 1.6
    COMMAND ${JAVA_ARCHIVE} -cvf ${SWIG_JAVA_JAR_NAME}
            org/opensim/modeling/*.class
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/src
    COMMENT "Compiling Java sources and creating jar archive."
    )

# Umbrella target for assembling the java bindings in the build tree.
# -------------------------------------------------------------------
add_custom_target(JavaBindings ALL
    DEPENDS "${SWIG_JAVA_JAR_BUILD_OUTPUT_PATH}"
    )

add_dependencies(JavaBindings osimJavaJNI)

set_target_properties(JavaBindings PROPERTIES
    PROJECT_LABEL "Java - umbrella target"
    FOLDER "Bindings")

# Install.
# --------
install(FILES "${SWIG_JAVA_JAR_BUILD_OUTPUT_PATH}"
    DESTINATION ${OPENSIM_INSTALL_JAVAJARDIR})


# Java tests.
# -----------

if(BUILD_TESTING)
    # Now actually add the tests.
    add_subdirectory(tests)
endif()

# MATLAB tests and installing.
# ----------------------------
add_subdirectory(Matlab)
