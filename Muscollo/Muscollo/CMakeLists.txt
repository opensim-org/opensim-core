
add_library(osimMuscollo
        InverseMuscleSolver.h            InverseMuscleSolver.cpp
        GlobalStaticOptimization.h       GlobalStaticOptimization.cpp
        INDYGO.h                         INDYGO.cpp
        DeGrooteFregly2016Muscle.h
        InverseMuscleSolverMotionData.h  InverseMuscleSolverMotionData.cpp
        )
target_link_libraries(osimMuscollo osimTools tropter)
# Do not generate warnings from OpenSim headers ("SYSTEM").
# No longer need to explicitly include OpenSim's headers;
# this is handled by linking to OpenSim's libraries.
# TODO target_include_directories(osimMuscollo
# TODO         SYSTEM PUBLIC ${OpenSim_INCLUDE_DIRS})
target_include_directories(osimMuscollo INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        # TODO this variable doesn't exist yet:
        $<INSTALL_INTERFACE:${MUSCLEREDUNDANCYSOLVER_INSTALL_INCLUDEDIR}>)

add_executable(opensim-muscollo opensim-muscollo.cpp)
target_link_libraries(opensim-muscollo osimMuscollo)

install(TARGETS opensim-muscollo DESTINATION bin)
set_target_properties(opensim-muscollo PROPERTIES
        DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})