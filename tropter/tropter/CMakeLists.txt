add_library(tropter
        tropter.h
        common.h
        Exception.h Exception.hpp Exception.cpp
        EigenUtilities.h
        optimization/AbstractOptimizationProblem.h
        optimization/OptimizationProblem.h
        optimization/OptimizationProblem.cpp
        optimization/OptimizationProblemDecorator_double.cpp
        optimization/OptimizationProblemDecorator_adouble.cpp
        optimization/OptimizationSolver.h
        optimization/OptimizationSolver.cpp
        optimization/SNOPTSolver.h
        optimization/SNOPTSolver.cpp
        optimization/IPOPTSolver.h
        optimization/IPOPTSolver.cpp
        optimalcontrol/DirectCollocation.h
        optimalcontrol/DirectCollocation.hpp
        optimalcontrol/DirectCollocation.cpp
        optimalcontrol/OptimalControlProblem.h
        optimalcontrol/OptimalControlProblem.hpp
        optimalcontrol/OptimalControlProblem.cpp
        optimalcontrol/OptimalControlIterate.h
        optimalcontrol/OptimalControlIterate.cpp
        )

# TODO generate_export_header() for exporting symbols on Windows.

# Dependencies.
# -------------
# PUBLIC means the headers are used in both of the following scenarios:
#   a. when building the library ("PRIVATE"), and
#   b. when a client library links to this library ("INTERFACE").
# SYSTEM means ignore compiler warnings; Ipopt presents warnings that we want
# to avoid for our own code.
#target_link_libraries(tropter PRIVATE fmt::fmt)

target_link_libraries(tropter PUBLIC Eigen3::Eigen)
# This takes care of includes.

# This was from when we used a Module file for ColPack:
#target_include_directories(tropter SYSTEM PRIVATE ${COLPACK_INCLUDE_DIR})
#target_link_libraries(tropter PRIVATE ${COLPACK_LIBRARIES})
target_link_libraries(tropter PRIVATE ColPack_static)

target_include_directories(tropter SYSTEM PUBLIC ${ADOLC_INCLUDES})
target_link_libraries(tropter PUBLIC ${ADOLC_LIBRARIES})

if(OPENMP_FOUND)
    # Let clients know that tropter is using OpenMP (PUBLIC). They don't need
    # use the OpenMP flag themselves, though.
    target_compile_definitions(tropter PUBLIC TROPTER_WITH_OPENMP)
    target_compile_options(tropter BEFORE PRIVATE
            ${OpenMP_CXX_FLAGS})
    if(NOT MSVC)
        target_link_libraries(tropter PRIVATE ${OpenMP_CXX_FLAGS})
    endif()
endif()

target_include_directories(tropter SYSTEM PRIVATE ${IPOPT_INCLUDE_DIRS})
target_link_libraries(tropter PRIVATE ${IPOPT_LIBRARIES})
if(LINUX)
    # On Ubuntu, this is useful for setting the HAVE_CSTDDEF definition.
    # We only include Ipopt in cpp files, so our clients don't need
    # this flag.
    target_compile_options(tropter PRIVATE ${IPOPT_CFLAGS_OTHER})
endif()
# TODO IPOPT_DEFINITIONS, IPOPT_LINK_FLAGS

if(TROPTER_WITH_SNOPT)
    target_link_libraries(tropter PUBLIC snopt7_cpp) # PRIVATE
endif()

if(WIN32)
    add_dependencies(tropter Copy_ADOLC_DLLs Copy_IPOPT_DLLs)
    #add_dependencies(tropter Copy_COLPACK_DLLs Copy_ADOLC_DLLs Copy_IPOPT_DLLs)
endif()

# Target properties, etc.
# -----------------------

if(MSVC)
    # Avoid warnings from our use of std::copy().
    target_compile_options(tropter PRIVATE
        "$<$<CONFIG:Debug>:/wd4996>")
endif()

# Only use the parts of Eigen that do not have a viral license.
# Also, make sure client projects remain non-viral ("PUBLIC").
target_compile_definitions(tropter PUBLIC EIGEN_MPL2_ONLY)
if(TROPTER_WITH_SNOPT)
    target_compile_definitions(tropter PUBLIC TROPTER_WITH_SNOPT)
endif()

# Specify the include directories that the tropter target and client targets
# should use.
target_include_directories(tropter PUBLIC
        # Used by other targets in this project:
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/tropter>
        # Used by client projects using an installed version of Tropter.
        $<INSTALL_INTERFACE:${TROPTER_INSTALL_INCLUDEDIR}>)

set_target_properties(tropter PROPERTIES
        PROJECT_LABEL "Library - tropter"
        FOLDER "tropter")

# Installing.
# -----------

install(TARGETS tropter EXPORT tropter-config # TODO change to tropter-targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION ${TROPTER_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h*")
