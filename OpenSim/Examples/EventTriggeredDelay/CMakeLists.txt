PROJECT(EventTriggeredDelay)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.6)

# Settings.
# ---------
SET(TARGET Main CACHE STRING "Name of example to build")


#SET(BASE_DIR "${PROJECT_SOURCE_DIR}")
#SET(INI_PATH "${PROJECT_SOURCE_DIR}/data/setup.ini")
#SET (CMAKE_EXTRA_INCLUDE_FILES src/Settings.h)
#configure_file ("${PROJECT_SOURCE_DIR}/src/Settings.h.in" "${PROJECT_SOURCE_DIR}/src/Settings.h")
#SET (CMAKE_EXTRA_INCLUDE_FILES)

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
# Identify the cpp file(s) that were to be built
FILE(GLOB SOURCE_FILES src/*.h src/*.cpp)
SET(SOURCE ${SOURCE_FILES})

ADD_EXECUTABLE(${TARGET} ${SOURCE_FILES})

TARGET_LINK_LIBRARIES(${TARGET} ${OpenSim_LIBRARIES})

SET_TARGET_PROPERTIES(${TARGET} PROPERTIES PROJECT_LABEL "Examples - EventTriggeredDelay")

# Configure installation.
# -----------------------
INSTALL_TARGETS(/bin ${TARGET})

# This block copies the additional files into the running directory
#file(COPY data DESTINATION ${CMAKE_BINARY_DIR}/Release)
#file(COPY data DESTINATION ${CMAKE_BINARY_DIR}/RelWithDebInfo)
#file(COPY data DESTINATION ${CMAKE_BINARY_DIR})
