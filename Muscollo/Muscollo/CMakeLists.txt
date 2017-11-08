
add_library(osimMuscollo SHARED
        osimMuscollo.h
        osimMuscolloDLL.h
        RegisterTypes_osimMuscollo.h
        RegisterTypes_osimMuscollo.cpp
        MuscolloUtilities.h
        MuscolloUtilities.cpp
        MucoTool.h
        MucoTool.cpp
        MucoProblem.h
        MucoProblem.cpp
        MucoCost.h
        MucoCost.cpp
        MucoStateTrackingCost.h
        MucoStateTrackingCost.cpp
        MucoSolver.h
        MucoSolver.cpp
        MucoIterate.h
        MucoIterate.cpp
        MucoTropterSolver.h
        MucoTropterSolver.cpp
        InverseMuscleSolver/InverseMuscleSolver.h
        InverseMuscleSolver/InverseMuscleSolver.cpp
        InverseMuscleSolver/GlobalStaticOptimization.h
        InverseMuscleSolver/GlobalStaticOptimization.cpp
        InverseMuscleSolver/INDYGO.h
        InverseMuscleSolver/INDYGO.cpp
        InverseMuscleSolver/DeGrooteFregly2016Muscle.h
        InverseMuscleSolver/InverseMuscleSolverMotionData.h
        InverseMuscleSolver/InverseMuscleSolverMotionData.cpp
        )
target_link_libraries(osimMuscollo PUBLIC osimActuators
                                   PRIVATE tropter)
								   
# TODO: OpenSim should link Simbody libraries automatically
target_link_libraries(osimMuscollo PUBLIC SimTKsimbody)

target_include_directories(osimMuscollo INTERFACE
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/Muscollo>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

set_target_properties(osimMuscollo PROPERTIES
        DEFINE_SYMBOL OSIMMUSCOLLO_EXPORTS
        PROJECT_LABEL "Library - osimMuscollo"
        FOLDER "Muscollo")

if(MUSCOLLO_COPY_DEPENDENCIES)
    if(APPLE)
        set(rpath_macro "\@loader_path")
    elseif(UNIX)
        set(rpath_macro "\$ORIGIN")
    endif()
    set_property(TARGET osimMuscollo APPEND PROPERTY
            INSTALL_RPATH "${rpath_macro}/")
endif()

if(WIN32)
   add_dependencies(osimMuscollo Copy_OpenSim_DLLs)
endif()

install(TARGETS osimMuscollo EXPORT MuscolloTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# TODO do not install InverseMuscleSolverMotionData.h b/c it uses Eigen.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION ${MUSCOLLO_INSTALL_INCLUDEDIR}/Muscollo
        FILES_MATCHING PATTERN "*.h*")
