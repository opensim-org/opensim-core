PROJECT (OptimizationExample)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.6)

# Settings.
# ---------
SET(TARGET optimizationExample CACHE TYPE STRING)
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
FILE(GLOB SOURCE_FILES *.h *.cpp)

ADD_EXECUTABLE(${TARGET} ${SOURCE_FILES})

TARGET_LINK_LIBRARIES(${TARGET} ${OpenSim_LIBRARIES})

# This block copies the additional files into the running directory
# For example vtp, obj files. Add to the end for more extentions
FILE(GLOB DATA_FILES *.vtp *.obj *.osim)
FOREACH (dataFile ${DATA_FILES})
    ADD_CUSTOM_COMMAND(
        TARGET ${TARGET}
        COMMAND ${CMAKE_COMMAND}
        ARGS -E copy
        ${dataFile}
        ${CMAKE_BINARY_DIR})
ENDFOREACH (dataFile) 
