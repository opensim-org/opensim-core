###########################################
## For building with the rest of OpenSim ##
###########################################
if(BUILD_API_EXAMPLES)

set(EXAMPLE_EXPRESSION_REPORTER_PLUGIN exampleExpressionReporterPlugin)

add_library(${EXAMPLE_EXPRESSION_REPORTER_PLUGIN} SHARED
    osimExpPluginDLL.cpp
    osimExpPluginDLL.h
    RegisterTypes_osimExpPlugin.cpp
    RegisterTypes_osimExpPlugin.h
    SymbolicExpressionReporter.cpp
    SymbolicExpressionReporter.h
    )
target_link_libraries(${EXAMPLE_EXPRESSION_REPORTER_PLUGIN}
    osimSimulation osimActuators osimCommon osimAnalyses osimTools osimLepton
    ${Simbody_LIBRARIES}
)
set_target_properties(${EXAMPLE_EXPRESSION_REPORTER_PLUGIN}
    PROPERTIES
    DEFINE_SYMBOL OSIMEXPPLUGIN_EXPORTS
    FOLDER "Examples"
    RUNTIME_OUTPUT_DIRECTORY
    "${OpenSim_BINARY_DIR}/OpenSim/Examples/SymbolicExpressionReporter"
)

################################################
## For building with existing OpenSim install ##
################################################
else(BUILD_API_EXAMPLES)

project(ExpressionReporterPlugin)

cmake_minimum_required(VERSION 3.2)

set(KIT plugin)
set(UKIT PLUGIN)
set(PLUGIN_NAME "osimExpressionReporter")

# Settings.
# ---------
# OpenSim uses C++20 language features.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find and hook up to OpenSim.
# ----------------------------
find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

# Configure this project.
# -----------------------
file(GLOB SOURCE_FILES *.cpp)
file(GLOB INCLUDE_FILES *.h)
add_library(${PLUGIN_NAME} SHARED ${SOURCE_FILES} ${INCLUDE_FILES})

target_link_libraries(${PLUGIN_NAME} ${OpenSim_LIBRARIES})

set(EXPORT_MACRO OSIMEXPPLUGIN_EXPORTS)
set_target_properties(
    ${PLUGIN_NAME} PROPERTIES
    DEFINE_SYMBOL ${EXPORT_MACRO}
    PROJECT_LABEL "Libraries - ${PLUGIN_NAME}")

# Configure installation.
# -----------------------
if(WIN32)
    install_targets(/sdk/lib/ RUNTIME_DIRECTORY /bin ${PLUGIN_NAME})
else(WIN32)
    install_targets(/bin/ RUNTIME_DIRECTORY /bin ${PLUGIN_NAME})
endif(WIN32)

endif(BUILD_API_EXAMPLES)
