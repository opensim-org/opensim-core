
add_library(osimMuscollo
        MucoProblem.h                    MucoProblem.cpp
        InverseMuscleSolver.h            InverseMuscleSolver.cpp
        GlobalStaticOptimization.h       GlobalStaticOptimization.cpp
        INDYGO.h                         INDYGO.cpp
        DeGrooteFregly2016Muscle.h
        InverseMuscleSolverMotionData.h  InverseMuscleSolverMotionData.cpp
        )
target_link_libraries(osimMuscollo PUBLIC osimActuators
                                   PRIVATE tropter)
# Do not generate warnings from OpenSim headers ("SYSTEM").
# No longer need to explicitly include OpenSim's headers;
# this is handled by linking to OpenSim's libraries.
# TODO target_include_directories(osimMuscollo
# TODO         SYSTEM PUBLIC ${OpenSim_INCLUDE_DIRS})
target_include_directories(osimMuscollo INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

set_target_properties(osimMuscollo PROPERTIES
        PROJECT_LABEL "Library - osimMuscollo"
        FOLDER "Muscollo")

if(WIN32)
    add_dependencies(osimMuscollo Copy_OpenSim_DLLs)
endif()

install(TARGETS osimMuscollo # EXPORT OpenSimTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# TODO do not install InverseMuscleSolverMotionData.h b/c it uses Eigen.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION ${MUSCOLLO_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h*")
