###########################################
## For building with the rest of OpenSim ##
###########################################
IF(BUILD_API_EXAMPLES)

SET(EXAMPLE_EXPRESSION_REPORTER_PLUGIN exampleExpressionReporterPlugin)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR}
            ${OpenSim_SOURCE_DIR}/OpenSim
            ${OpenSim_SOURCE_DIR}/Vendors
            ${OpenSim_SOURCE_DIR}/Vendors/lepton/include
            )

ADD_LIBRARY(${EXAMPLE_EXPRESSION_REPORTER_PLUGIN} SHARED
    osimExpPluginDLL.cpp
    osimExpPluginDLL.h
    RegisterTypes_osimExpPlugin.cpp
    RegisterTypes_osimExpPlugin.h
    SymbolicExpressionReporter.cpp
    SymbolicExpressionReporter.h
    )
TARGET_LINK_LIBRARIES(${EXAMPLE_EXPRESSION_REPORTER_PLUGIN}
    osimSimulation osimActuators osimCommon osimAnalyses osimTools osimLepton
    ${SIMTK_ALL_LIBS}
)
SET_TARGET_PROPERTIES(${EXAMPLE_EXPRESSION_REPORTER_PLUGIN}
    PROPERTIES
    DEFINE_SYMBOL OSIMEXPPLUGIN_EXPORTS
    PROJECT_LABEL "Examples - ${EXAMPLE_EXPRESSION_REPORTER_PLUGIN}"
    RUNTIME_OUTPUT_DIRECTORY
    "${OpenSim_BINARY_DIR}/OpenSim/Examples/SymbolicExpressionReporter"
)

################################################
## For building with existing OpenSim install ##
################################################
ELSE(BUILD_API_EXAMPLES)

PROJECT(ExpressionReporterPlugin)

cmake_minimum_required(VERSION 2.8.6)

SET(KIT plugin)
SET(UKIT PLUGIN)

FILE(GLOB SOURCE_FILES *.cpp)
FILE(GLOB INCLUDE_FILES *.h)

SET(OPENSIM_INSTALL_DIR $ENV{OPENSIM_HOME}
        CACHE PATH "Top-level directory of OpenSim install")

SET(PLUGIN_NAME "osimExpressionReporter")

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
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++")
        endif()
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()
endif()

FIND_PACKAGE(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

INCLUDE_DIRECTORIES(${OpenSim_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenSim_LIB_DIR})

ADD_LIBRARY(${PLUGIN_NAME} SHARED ${SOURCE_FILES} ${INCLUDE_FILES})

TARGET_LINK_LIBRARIES(${PLUGIN_NAME} ${OpenSim_LIBRARIES})


SET(EXPORT_MACRO OSIMEXPPLUGIN_EXPORTS)
SET_TARGET_PROPERTIES(
    ${PLUGIN_NAME} PROPERTIES
    DEFINE_SYMBOL ${EXPORT_MACRO}
    PROJECT_LABEL "Libraries - ${PLUGIN_NAME}")

IF(WIN32)
    INSTALL_TARGETS(/sdk/lib/ RUNTIME_DIRECTORY /bin ${PLUGIN_NAME})
ELSE(WIN32)
    INSTALL_TARGETS(/bin/ RUNTIME_DIRECTORY /bin ${PLUGIN_NAME})
ENDIF(WIN32)

ENDIF(BUILD_API_EXAMPLES)
