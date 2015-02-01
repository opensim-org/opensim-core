PROJECT(Test)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.6)

# Settings.
# ---------
SET(TARGET test CACHE STRING "Name of example to build")
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
SET(SOURCE_FILES ${TARGET}.cpp) 

ADD_EXECUTABLE(${TARGET}  ${SOURCE_FILES})

TARGET_LINK_LIBRARIES(${TARGET} ${OpenSim_LIBRARIES})

SET_TARGET_PROPERTIES(
    ${TARGET} PROPERTIES
    PROJECT_LABEL "Test Environment")


# Configure installation.
# -----------------------
INSTALL_TARGETS(/bin ${TARGET})
