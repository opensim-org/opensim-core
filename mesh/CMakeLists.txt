project(mesh) # TODO another name: Trop (trajectory optimization)
# TODO TROP
cmake_minimum_required(VERSION 3.2)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(IPOPT REQUIRED ipopt)
pkg_check_modules(ADOLC REQUIRED adolc)

link_directories(${IPOPT_LIBRARY_DIRS})
link_directories(${ADOLC_LIBRARY_DIRS})

add_executable(mesh mesh.cpp)
# PUBLIC means the headers are used in both of the following scenarios:
#   a. when building the library ("PRIVATE"), and 
#   b. when a client library links to this library ("INTERFACE").
# SYSTEM means ignore compiler warnings; Ipopt presents warnings that we want
# to avoid for our own code.
target_include_directories(mesh        PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(mesh SYSTEM PUBLIC ${IPOPT_INCLUDE_DIRS})
target_include_directories(mesh SYSTEM PUBLIC ${ADOLC_INCLUDE_DIRS})
target_link_libraries(mesh ${IPOPT_LIBRARIES} ${ADOLC_LIBRARIES})

# TODO set(CMAKE_CXX_FLAGS -DEIGEN_MPL2_ONLY)

