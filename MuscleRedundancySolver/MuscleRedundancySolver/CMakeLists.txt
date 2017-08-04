
add_library(osimMuscleRedundancySolver
        InverseMuscleSolver.h            InverseMuscleSolver.cpp
        GlobalStaticOptimizationSolver.h GlobalStaticOptimizationSolver.cpp
        MuscleRedundancySolver.h         MuscleRedundancySolver.cpp
        DeGroote2016Muscle.h
        InverseMuscleSolverMotionData.h InverseMuscleSolverMotionData.cpp
        )
target_link_libraries(osimMuscleRedundancySolver osimTools mesh)
# Do not generate warnings from OpenSim headers ("SYSTEM").
# No longer need to explicitly include OpenSim's headers;
# this is handled by linking to OpenSim's libraries.
# TODO target_include_directories(osimMuscleRedundancySolver
# TODO         SYSTEM PUBLIC ${OpenSim_INCLUDE_DIRS})
target_include_directories(osimMuscleRedundancySolver INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        # TODO this variable doesn't exist yet:
        $<INSTALL_INTERFACE:${MUSCLEREDUNDANCYSOLVER_INSTALL_INCLUDEDIR}>)

add_executable(opensim-mrs opensim-mrs.cpp)
target_link_libraries(opensim-mrs osimMuscleRedundancySolver)
