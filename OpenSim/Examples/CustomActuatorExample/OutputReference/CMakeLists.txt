PROJECT (CustomActuatorExample)

cmake_minimum_required(VERSION 2.8.6)

FILE(GLOB SOURCE_FILES *.cpp )
FILE(GLOB INCLUDE_FILES *.h ) 

# Settings.
# ---------
SET(TARGET toyLeg_example
		CACHE STRING "Name of example to build")
SET(OPENSIM_INSTALL_DIR $ENV{OPENSIM_HOME} 
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
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++")
        endif()
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()
endif()

FIND_PACKAGE(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

INCLUDE_DIRECTORIES(${OpenSim_INCLUDE_DIRS})
LINK_DIRECTORIES(${OpenSim_LIB_DIR})

# Configure this project.
# -----------------------
ADD_EXECUTABLE(${TARGET}  ${SOURCE_FILES} ${INCLUDE_FILES}) 

TARGET_LINK_LIBRARIES(${TARGET} ${OpenSim_LIBRARIES})

SET_TARGET_PROPERTIES(
	${TARGET} PROPERTIES
	PROJECT_LABEL "Actuator_examples")

INSTALL_TARGETS(/ ${TARGET})

