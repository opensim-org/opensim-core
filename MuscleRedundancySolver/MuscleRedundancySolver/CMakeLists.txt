
add_library(osimMuscleRedundancySolver
        MuscleRedundancySolver.h MuscleRedundancySolver.cpp
        GlobalStaticOptimizationSolver.h GlobalStaticOptimizationSolver.cpp
        DeGroote2016Muscle.h
        InverseMuscleSolverMotionData.h InverseMuscleSolverMotionData.cpp
        )
target_link_libraries(osimMuscleRedundancySolver osimTools mesh)
# Do not generate warnings from OpenSim headers ("SYSTEM").
target_include_directories(osimMuscleRedundancySolver
        SYSTEM PUBLIC ${OpenSim_INCLUDE_DIRS})
target_include_directories(osimMuscleRedundancySolver INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        # TODO this variable doesn't exist yet:
        $<INSTALL_INTERFACE:${MUSCLEREDUNDANCYSOLVER_INSTALL_INCLUDEDIR}>)

add_executable(opensim-mrs opensim-mrs.cpp)
target_link_libraries(opensim-mrs osimMuscleRedundancySolver)
