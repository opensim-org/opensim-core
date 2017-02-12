option(TOMU_WITH_SNOPT
        "Compile support for the SNOPT optimization library; \
        requires Fortran compiler"
        OFF)
if(TOMU_WITH_SNOPT)
    project(tomu C CXX Fortran) # TODO Fortran only for SNOPT interface.
else()
    project(tomu C CXX)
endif()
cmake_minimum_required(VERSION 3.2)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Weverything")
# TODO treat Ipopt and such as system headers to ignore their warnings.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

enable_testing()

add_subdirectory(mesh)


