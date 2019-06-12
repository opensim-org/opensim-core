
set(MOCO_SOURCES
        osimMoco.h
        osimMocoDLL.h
        RegisterTypes_osimMoco.h
        RegisterTypes_osimMoco.cpp
        MocoUtilities.h
        MocoUtilities.cpp
        MocoStudy.h
        MocoStudy.cpp
        MocoBounds.h
        MocoBounds.cpp
        MocoVariableInfo.h
        MocoVariableInfo.cpp
        MocoProblem.h
        MocoProblem.cpp
        MocoProblemInfo.h
        MocoProblemRep.h
        MocoProblemRep.cpp
        MocoCost/MocoCost.h
        MocoCost/MocoCost.cpp
        MocoCost/MocoMarkerEndpointCost.h
        MocoCost/MocoMarkerEndpointCost.cpp
        MocoCost/MocoMarkerTrackingCost.h
        MocoCost/MocoMarkerTrackingCost.cpp
        MocoWeightSet.h
        MocoCost/MocoStateTrackingCost.h
        MocoCost/MocoStateTrackingCost.cpp
        MocoCost/MocoControlCost.h
        MocoCost/MocoControlCost.cpp
        MocoCost/MocoControlTrackingCost.h
        MocoCost/MocoControlTrackingCost.cpp
        MocoCost/MocoJointReactionCost.h
        MocoCost/MocoJointReactionCost.cpp
        MocoCost/MocoOrientationTrackingCost.h
        MocoCost/MocoOrientationTrackingCost.cpp
        MocoCost/MocoTranslationTrackingCost.h
        MocoCost/MocoTranslationTrackingCost.cpp
        MocoSolver.h
        MocoSolver.cpp
        MocoDirectCollocationSolver.h
        MocoDirectCollocationSolver.cpp
        MocoIterate.h
        MocoIterate.cpp
        MocoTropterSolver.h
        MocoTropterSolver.cpp
        MocoParameter.h
        MocoParameter.cpp
        MocoConstraint.h
        MocoConstraint.cpp
        MocoControlBoundConstraint.cpp
        MocoControlBoundConstraint.h
        InverseMuscleSolver/InverseMuscleSolver.h
        InverseMuscleSolver/InverseMuscleSolver.cpp
        InverseMuscleSolver/GlobalStaticOptimization.h
        InverseMuscleSolver/GlobalStaticOptimization.cpp
        InverseMuscleSolver/INDYGO.h
        InverseMuscleSolver/INDYGO.cpp
        InverseMuscleSolver/INDYGOProblemInternal.h
        InverseMuscleSolver/DeGrooteFregly2016MuscleStandalone.h
        InverseMuscleSolver/InverseMuscleSolverMotionData.h
        InverseMuscleSolver/InverseMuscleSolverMotionData.cpp
        Components/DeGrooteFregly2016Muscle.h
        Components/DeGrooteFregly2016Muscle.cpp
        Components/ActivationCoordinateActuator.h
        Components/StationPlaneContactForce.h
        Components/StationPlaneContactForce.cpp
        Components/PositionMotion.h
        Components/PositionMotion.cpp
        Components/ModelFactory.h
        Components/ModelFactory.cpp
        Components/DiscreteForces.h
        Components/DiscreteForces.cpp
        Components/AccelerationMotion.h
        Components/AccelerationMotion.cpp
        MocoCasADiSolver/MocoCasADiSolver.h
        MocoCasADiSolver/MocoCasADiSolver.cpp
        MocoCasADiSolver/MocoCasOCProblem.h
        MocoCasADiSolver/MocoCasOCProblem.cpp
        MocoCasADiSolver/CasOCProblem.h
        MocoCasADiSolver/CasOCProblem.cpp
        MocoCasADiSolver/CasOCSolver.h
        MocoCasADiSolver/CasOCSolver.cpp
        MocoCasADiSolver/CasOCFunction.h
        MocoCasADiSolver/CasOCFunction.cpp
        MocoCasADiSolver/CasOCTranscription.h
        MocoCasADiSolver/CasOCTranscription.cpp
        MocoCasADiSolver/CasOCTrapezoidal.h
        MocoCasADiSolver/CasOCTrapezoidal.cpp
        MocoCasADiSolver/CasOCHermiteSimpson.h
        MocoCasADiSolver/CasOCHermiteSimpson.cpp
        MocoCasADiSolver/CasOCIterate.h
        MocoInverse.cpp
        MocoInverse.h
        Common/TableProcessor.h
        ModelProcessor.h
        ModelOperators.h
        MocoTool.cpp
        MocoTool.h
        )
if (MOCO_WITH_TROPTER)
    list(APPEND MOCO_SOURCES
            tropter/TropterProblem.h
            tropter/TropterProblem.cpp)
endif ()
add_library(osimMoco SHARED ${MOCO_SOURCES})
target_link_libraries(osimMoco PUBLIC osimTools
        PRIVATE tropter casadi)

target_include_directories(osimMoco INTERFACE
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/Moco>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

set_target_properties(osimMoco PROPERTIES
        DEFINE_SYMBOL OSIMMOCO_EXPORTS
        PROJECT_LABEL "Library - osimMoco"
        FOLDER "Moco"
        )
if (MOCO_WITH_TROPTER)
    target_compile_definitions(osimMoco PUBLIC MOCO_WITH_TROPTER)
endif ()

if (MOCO_COPY_DEPENDENCIES)
    if (APPLE)
        set(rpath_macro "\@loader_path")
    elseif (UNIX)
        set(rpath_macro "\$ORIGIN")
    endif ()
    set_property(TARGET osimMoco APPEND PROPERTY
            INSTALL_RPATH "${rpath_macro}/")
endif ()

if (WIN32)
    add_dependencies(osimMoco Copy_OpenSim_DLLs)
endif ()

install(TARGETS osimMoco EXPORT MocoTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# TODO do not install InverseMuscleSolverMotionData.h b/c it uses Eigen.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION ${MOCO_INSTALL_INCLUDEDIR}/Moco
        FILES_MATCHING PATTERN "*.h*")
