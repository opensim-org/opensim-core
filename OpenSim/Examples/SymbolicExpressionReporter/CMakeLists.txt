###########################################
## For building with the rest of OpenSim ##
###########################################
if(BUILD_API_EXAMPLES)

set(EXAMPLE_EXPRESSION_REPORTER_PLUGIN exampleExpressionReporterPlugin)

include_directories(${OpenSim_SOURCE_DIR}
            ${OpenSim_SOURCE_DIR}/OpenSim
            ${OpenSim_SOURCE_DIR}/Vendors
            ${OpenSim_SOURCE_DIR}/Vendors/lepton/include
            )

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
    PROJECT_LABEL "Examples - ${EXAMPLE_EXPRESSION_REPORTER_PLUGIN}"
    RUNTIME_OUTPUT_DIRECTORY
    "${OpenSim_BINARY_DIR}/OpenSim/Examples/SymbolicExpressionReporter"
)

################################################
## For building with existing OpenSim install ##
################################################
else(BUILD_API_EXAMPLES)

project(ExpressionReporterPlugin)

cmake_minimum_required(VERSION 2.8.6)

set(KIT plugin)
set(UKIT PLUGIN)
set(PLUGIN_NAME "osimExpressionReporter")

# Settings.
# ---------
set(OPENSIM_INSTALL_DIR $ENV{OPENSIM_HOME}
        CACHE PATH "Top-level directory of OpenSim install")


# Find and hook up to OpenSim.
# ----------------------------
# OpenSim uses C++11 language features.
if("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU" OR
        "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    # Using C++11 on OSX requires using libc++ instead of libstd++.
    # libc++ is an implementation of the C++ standard library for OSX.
    if(APPLE)
        if(XCODE)
            set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
            set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
        else()
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
            if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
            endif()
        endif()
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()
endif()

find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

include_directories(${OpenSim_INCLUDE_DIRS})
link_directories(${OpenSim_LIB_DIR})

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
