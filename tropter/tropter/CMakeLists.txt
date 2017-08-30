add_library(tropter
        tropter.h
        common.h
        optimization/AbstractOptimizationProblem.h
        optimization/OptimizationProblem.h
        optimization/OptimizationProblem.hpp
        optimization/OptimizationProblem.cpp
        optimization/OptimizationProblemProxy_double.cpp
        optimization/OptimizationProblemProxy_adouble.cpp
        optimization/OptimizationSolver.h
        optimization/OptimizationSolver.cpp
        optimization/SNOPTSolver.h
        optimization/SNOPTSolver.cpp
        optimization/IpoptSolver.h
        optimization/IpoptSolver.cpp
        optimalcontrol/DirectCollocation.h
        optimalcontrol/DirectCollocation.hpp
        optimalcontrol/DirectCollocation.cpp
        optimalcontrol/OptimalControlProblem.h
        optimalcontrol/OptimalControlIterate.h
        optimalcontrol/OptimalControlIterate.cpp
        legacy.h
        legacy.cpp
        )

# PUBLIC means the headers are used in both of the following scenarios:
#   a. when building the library ("PRIVATE"), and
#   b. when a client library links to this library ("INTERFACE").
# SYSTEM means ignore compiler warnings; Ipopt presents warnings that we want
# to avoid for our own code.
target_link_libraries(tropter Eigen3::Eigen) # This takes care of includes.

target_include_directories(tropter SYSTEM PUBLIC ${COLPACK_INCLUDE_DIR})
target_link_libraries(tropter ${COLPACK_LIBRARIES})

# This line was from when we used pkg-config to find ADOL-C.
#target_include_directories(tropter SYSTEM PUBLIC ${ADOLC_INCLUDE_DIRS})
target_include_directories(tropter SYSTEM PUBLIC ${ADOLC_INCLUDES})
target_link_libraries(tropter ${ADOLC_LIBRARIES})

target_include_directories(tropter SYSTEM PUBLIC ${IPOPT_INCLUDE_DIRS})
target_link_libraries(tropter ${IPOPT_LIBRARIES})

if(MUSCOLLO_WITH_SNOPT)
    target_link_libraries(tropter snopt7_cpp)
endif()
# TODO IPOPT_DEFINITIONS, IPOPT_LINK_FLAGS

# Only use the parts of Eigen that do not have a viral license.
# Also, make sure client projects remain non-viral ("PUBLIC").
target_compile_definitions(tropter PUBLIC EIGEN_MPL2_ONLY)
if(MUSCOLLO_WITH_SNOPT)
    target_compile_definitions(tropter PUBLIC MUSCOLLO_WITH_SNOPT)
endif()

# Specify the include directories that the tropter target and client targets
# should use.
target_include_directories(tropter PUBLIC
        # Used by other targets in this project:
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/tropter>
        # Used by client projects using an installed version of Tropter.
        # TODO use variable for the path.
        $<INSTALL_INTERFACE:${TROPTER_INSTALL_INCLUDEDIR}>)

set_target_properties(tropter PROPERTIES FOLDER "tropter")

if(WIN32)
    add_dependencies(tropter Copy_COLPACK_DLLs Copy_ADOLC_DLLs Copy_IPOPT_DLLs)
endif()

install(TARGETS tropter EXPORT tropter-config
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION ${TROPTER_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h*")




add_subdirectory(sandbox)