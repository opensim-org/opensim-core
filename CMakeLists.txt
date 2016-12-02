project(tomu)
cmake_minimum_required(VERSION 3.2)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Weverything")
# TODO treat Ipopt and such as system headers to ignore their warnings.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

add_subdirectory(mesh)


