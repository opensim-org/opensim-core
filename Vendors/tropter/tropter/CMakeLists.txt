add_library(tropter SHARED
        tropter.h
        common.h
        utilities.h utilities.cpp
        Exception.h Exception.hpp Exception.cpp
        EigenUtilities.h
        SparsityPattern.h
        SparsityPattern.cpp
        optimization/AbstractProblem.h
        optimization/AbstractProblem.cpp
        optimization/Problem.h
        optimization/Problem.cpp
        optimization/ProblemDecorator.h
        optimization/ProblemDecorator_double.h
        optimization/ProblemDecorator_double.cpp
        optimization/ProblemDecorator_adouble.h
        optimization/ProblemDecorator_adouble.cpp
        optimization/Solver.h
        optimization/Solver.cpp
        optimization/SNOPTSolver.h
        optimization/SNOPTSolver.cpp
        optimization/IPOPTSolver.h
        optimization/IPOPTSolver.cpp
        optimization/internal/GraphColoring.h
        optimization/internal/GraphColoring.cpp
        optimalcontrol/DirectCollocation.h
        optimalcontrol/DirectCollocation.hpp
        optimalcontrol/DirectCollocation.cpp
        optimalcontrol/Problem.h
        optimalcontrol/Problem.hpp
        optimalcontrol/Problem.cpp
        optimalcontrol/Iterate.h
        optimalcontrol/Iterate.cpp
        optimalcontrol/DirectCollocation.h
        optimalcontrol/DirectCollocation.hpp
        optimalcontrol/DirectCollocation.cpp
        optimalcontrol/transcription/Base.h
        optimalcontrol/transcription/Trapezoidal.h
        optimalcontrol/transcription/Trapezoidal.hpp
        optimalcontrol/transcription/Trapezoidal.cpp
        optimalcontrol/transcription/HermiteSimpson.h
        optimalcontrol/transcription/HermiteSimpson.hpp
        optimalcontrol/transcription/HermiteSimpson.cpp
        )

# TODO generate_export_header() for exporting symbols on Windows.

# Dependencies.
# -------------
# PUBLIC means the headers are used in both of the following scenarios:
#   a. when building the library ("PRIVATE"), and
#   b. when a client library links to this library ("INTERFACE").
# SYSTEM means ignore compiler warnings; Ipopt presents warnings that we want
# to avoid for our own code.

target_link_libraries(tropter PUBLIC optional-lite)

#target_link_libraries(tropter PRIVATE fmt::fmt)

target_link_libraries(tropter PUBLIC Eigen3::Eigen)
# This takes care of includes.

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

if(UNIX)
    target_link_libraries(tropter PRIVATE PkgConfig::IPOPT)
else()
    target_link_libraries(tropter PRIVATE ipopt)
endif()
if(LINUX)
    # On Ubuntu, this is useful for setting the HAVE_CSTDDEF definition.
    # We only include Ipopt in cpp files, so our clients don't need
    # this flag.
    target_compile_options(tropter PRIVATE ${IPOPT_CFLAGS_OTHER})
    # We must use this flag on Linux for tropter to link to gfortran.
    # In CMake 3.13, we could use target_link_options().
    set_target_properties(tropter PROPERTIES LINK_FLAGS ${IPOPT_LDFLAGS})
endif()
# TODO IPOPT_DEFINITIONS, IPOPT_LINK_FLAGS

if(TROPTER_WITH_SNOPT)
    target_link_libraries(tropter PRIVATE snopt7_cpp)
endif()

if(WIN32)
    add_dependencies(tropter Copy_ADOLC_DLLs Copy_IPOPT_DLLs)
    if(TROPTER_WITH_SNOPT)
        add_dependencies(tropter Copy_snopt7_DLLs)
    endif()
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
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
        # Used by client projects using an installed version of tropter.
        $<INSTALL_INTERFACE:${TROPTER_INSTALL_INCLUDEDIR}>)

set_target_properties(tropter PROPERTIES
        PROJECT_LABEL "Library - tropter"
        FOLDER "tropter"
        # Avoid need for __declspec(dllexport), etc.
        WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

# Installing.
# -----------

install(TARGETS tropter EXPORT tropter-config # TODO change to tropter-targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION ${TROPTER_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h*"
        PATTERN "*internal" EXCLUDE)
