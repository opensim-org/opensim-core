PROJECT (CustomActuatorExample)

cmake_minimum_required(VERSION 2.8.6)

file(GLOB SOURCE_FILES *.cpp )
file(GLOB INCLUDE_FILES *.h )

# Settings.
# ---------
set(TARGET toyLeg_example
        CACHE STRING "Name of example to build")
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
add_executable(${TARGET}  ${SOURCE_FILES} ${INCLUDE_FILES})

target_link_libraries(${TARGET} ${OpenSim_LIBRARIES})

set_target_properties(
    ${TARGET} PROPERTIES
    PROJECT_LABEL "Actuator_examples")

install_targets(/ ${TARGET})

