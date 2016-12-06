add_library(mesh mesh.h mesh.cpp)

# PUBLIC means the headers are used in both of the following scenarios:
#   a. when building the library ("PRIVATE"), and
#   b. when a client library links to this library ("INTERFACE").
# SYSTEM means ignore compiler warnings; Ipopt presents warnings that we want
# to avoid for our own code.
target_include_directories(mesh        PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(mesh SYSTEM PUBLIC ${IPOPT_INCLUDE_DIRS})
target_include_directories(mesh SYSTEM PUBLIC ${ADOLC_INCLUDE_DIRS})
target_link_libraries(mesh ${IPOPT_LIBRARIES} ${ADOLC_LIBRARIES})

target_include_directories(mesh INTERFACE
        # Used by other targets in this project:
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        # Used by client projects using an installed version of MESH.
        # TODO use variable for the path.
        $<INSTALL_INTERFACE:${MESH_INSTALL_INCLUDEDIR}>)
