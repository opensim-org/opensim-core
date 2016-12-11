project(tomu C CXX Fortran) # TODO Fortran only for SNOPT interface.
cmake_minimum_required(VERSION 3.2)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Weverything")
# TODO treat Ipopt and such as system headers to ignore their warnings.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

enable_testing()

add_subdirectory(mesh)


