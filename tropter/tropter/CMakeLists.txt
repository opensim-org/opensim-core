add_library(tropter
        tropter.h
        common.h
        optimization/AbstractOptimizationProblem.h
        optimization/OptimizationProblem.h
        optimization/OptimizationProblem.cpp
        optimization/OptimizationProblemDecorator_double.cpp
        optimization/OptimizationProblemDecorator_adouble.cpp
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
        )

# PUBLIC means the headers are used in both of the following scenarios:
#   a. when building the library ("PRIVATE"), and
#   b. when a client library links to this library ("INTERFACE").
# SYSTEM means ignore compiler warnings; Ipopt presents warnings that we want
# to avoid for our own code.
target_link_libraries(tropter PUBLIC Eigen3::Eigen)
# This takes care of includes.

target_include_directories(tropter SYSTEM PUBLIC ${COLPACK_INCLUDE_DIR})
target_link_libraries(tropter PUBLIC ${COLPACK_LIBRARIES})

# This line was from when we used pkg-config to find ADOL-C:
#target_include_directories(tropter SYSTEM PUBLIC ${ADOLC_INCLUDE_DIRS})
target_include_directories(tropter SYSTEM PUBLIC ${ADOLC_INCLUDES})
target_link_libraries(tropter PUBLIC ${ADOLC_LIBRARIES})

if(OPENMP_FOUND)
    target_compile_definitions(tropter PUBLIC TROPTER_WITH_OPENMP)
    target_compile_options(tropter BEFORE PUBLIC
            $<$<COMPILE_LANGUAGE:C>:${OpenMP_C_FLAGS}>
            $<$<COMPILE_LANGUAGE:CXX>:${OpenMP_CXX_FLAGS}>)
    if(APPLE AND CMAKE_CXX_COMPILER_ID MATCHES Clang)
        # Using LLVM Clang on Macs: having trouble finding the OpenMP library.
        target_link_libraries(tropter PUBLIC iomp5)
    endif()
endif()

target_include_directories(tropter SYSTEM PUBLIC ${IPOPT_INCLUDE_DIRS})
target_link_libraries(tropter PUBLIC ${IPOPT_LIBRARIES})
# TODO IPOPT_DEFINITIONS, IPOPT_LINK_FLAGS

if(MUSCOLLO_WITH_SNOPT)
    target_link_libraries(tropter snopt7_cpp)
endif()

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
