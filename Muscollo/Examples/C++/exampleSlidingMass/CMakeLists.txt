cmake_minimum_required(VERSION 3.2)
project(Muscollo_exampleSlidingMass)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    # This project is standalone (not part of the Muscollo source distribution).
    # TODO
    find_package(Muscollo REQUIRED)
    # instead: find_package(OpenSim REQUIRED)
endif()

add_executable(exampleSlidingMass exampleSlidingMass.cpp)
target_link_libraries(exampleSlidingMass osimMuscollo)
