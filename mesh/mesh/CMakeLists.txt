add_library(mesh
    mesh.h
    common.h
    OptimizationProblem.h   OptimizationProblem.hpp OptimizationProblem.cpp
    OptimizationSolver.h                            OptimizationSolver.cpp
    SNOPTSolver.h                                   SNOPTSolver.cpp
    IpoptSolver.h                                   IpoptSolver.cpp
    DirectCollocation.h     DirectCollocation.hpp   DirectCollocation.cpp
    OptimalControlProblem.h
    legacy.h                                        legacy.cpp
    )

# PUBLIC means the headers are used in both of the following scenarios:
#   a. when building the library ("PRIVATE"), and
#   b. when a client library links to this library ("INTERFACE").
# SYSTEM means ignore compiler warnings; Ipopt presents warnings that we want
# to avoid for our own code.
target_link_libraries(mesh Eigen3::Eigen) # This takes care of includes.
target_include_directories(mesh SYSTEM PUBLIC ${IPOPT_INCLUDE_DIRS})
# This line was from when we used pkg-config to find ADOL-C.
#target_include_directories(mesh SYSTEM PUBLIC ${ADOLC_INCLUDE_DIRS})
target_include_directories(mesh SYSTEM PUBLIC ${ADOLC_INCLUDES})
target_link_libraries(mesh ${IPOPT_LIBRARIES} ${ADOLC_LIBRARIES})
if(TOMU_WITH_SNOPT)
    target_link_libraries(mesh snopt7_cpp)
endif()
# TODO IPOPT_DEFINITIONS, IPOPT_LINK_FLAGS

# Only use the parts of Eigen that do not have a viral license.
# Also, make sure client projects remain non-viral ("PUBLIC").
target_compile_definitions(mesh PUBLIC EIGEN_MPL2_ONLY)
if(TOMU_WITH_SNOPT)
    target_compile_definitions(mesh PUBLIC TOMU_WITH_SNOPT)
endif()

# Specify the include directories that client targets should use.
target_include_directories(mesh INTERFACE
        # Used by other targets in this project:
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        # Used by client projects using an installed version of MESH.
        # TODO use variable for the path.
        $<INSTALL_INTERFACE:${MESH_INSTALL_INCLUDEDIR}>)