
set(OPENSIM_MOCO_MAJOR_VERSION 0)
set(OPENSIM_MOCO_MINOR_VERSION 5)
set(OPENSIM_MOCO_PATCH_VERSION 0)

set(OPENSIM_MOCO_RELEASE_VERSION
        "${OPENSIM_MOCO_MAJOR_VERSION}.${OPENSIM_MOCO_MINOR_VERSION}.${OPENSIM_MOCO_PATCH_VERSION}"
        )
# The OPENSIM_MOCO_QUALIFIED_VERSION is more granular than
# OPENSIM_MOCO_RELEASE_VERSION.
# If git is not found, we simply append "?" to the release version.
# If the checked-out commit is tagged, the qualified version is the tag name.
# If the checked-out commit is not tagged, the qualified version is
#   <release-version>-<git-commit-date>-<git-commit-short-hash>
#   For example: 0.5.0-2020-07-01-adba3310
set(MOCO_QUALIFIED_VERSION "")
if(Git_FOUND)
    # This command provides the annotated tag for the current commit, if it
    # exists, and returns an error if a tag does not exist.
    execute_process(
            COMMAND "${GIT_EXECUTABLE}" describe --exact-match
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            RESULT_VARIABLE GIT_DESCRIBE_EXACT_MATCH_RETVAL
            OUTPUT_VARIABLE GIT_TAG
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
    )
    if(${GIT_DESCRIBE_EXACT_MATCH_RETVAL} EQUAL 0)
        # A tag exists.
        set(MOCO_QUALIFIED_VERSION ${GIT_TAG})
        # Do not add anything to the version.
    else()
        execute_process(
                COMMAND "${GIT_EXECUTABLE}" log -1 --format=%h
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                OUTPUT_VARIABLE GIT_COMMIT_HASH
                OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        # Prepend date for sorting.
        execute_process(
                COMMAND "${GIT_EXECUTABLE}" show -s --format=%cd --date=short HEAD
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                OUTPUT_VARIABLE GIT_COMMIT_DATE
                OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        set(MOCO_QUALIFIED_VERSION
                "${OPENSIM_MOCO_RELEASE_VERSION}-${GIT_COMMIT_DATE}-${GIT_COMMIT_HASH}")
    endif()
else()
    set(OPENSIM_MOCO_QUALIFIED_VERSION "${OPENSIM_MOCO_RELEASE_VERSION}?")
endif()
set(OPENSIM_MOCO_QUALIFIED_VERSION "${MOCO_QUALIFIED_VERSION}" CACHE STRING
        "Qualified version string, containing git commit date/hash suffix." FORCE)

# TODO: Use OpenSimAddLibrary() macro.
set(MOCO_SOURCES
        osimMoco.h
        osimMocoDLL.h
        About.h
        About.cpp
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
        MocoGoal/MocoGoal.h
        MocoGoal/MocoGoal.cpp
        MocoGoal/MocoMarkerFinalGoal.h
        MocoGoal/MocoMarkerFinalGoal.cpp
        MocoGoal/MocoMarkerTrackingGoal.h
        MocoGoal/MocoMarkerTrackingGoal.cpp
        MocoWeightSet.h
        MocoGoal/MocoStateTrackingGoal.h
        MocoGoal/MocoStateTrackingGoal.cpp
        MocoGoal/MocoSumSquaredStateGoal.h
        MocoGoal/MocoSumSquaredStateGoal.cpp
        MocoGoal/MocoContactTrackingGoal.cpp
        MocoGoal/MocoContactTrackingGoal.h
        MocoGoal/MocoControlGoal.h
        MocoGoal/MocoControlGoal.cpp
        MocoGoal/MocoControlTrackingGoal.h
        MocoGoal/MocoControlTrackingGoal.cpp
        MocoGoal/MocoJointReactionGoal.h
        MocoGoal/MocoJointReactionGoal.cpp
        MocoGoal/MocoOrientationTrackingGoal.h
        MocoGoal/MocoOrientationTrackingGoal.cpp
        MocoGoal/MocoOutputGoal.h
        MocoGoal/MocoOutputGoal.cpp
        MocoGoal/MocoTranslationTrackingGoal.h
        MocoGoal/MocoTranslationTrackingGoal.cpp
        MocoGoal/MocoAngularVelocityTrackingGoal.h
        MocoGoal/MocoAngularVelocityTrackingGoal.cpp
        MocoGoal/MocoAccelerationTrackingGoal.h
        MocoGoal/MocoAccelerationTrackingGoal.cpp
        MocoGoal/MocoInitialActivationGoal.cpp
        MocoGoal/MocoInitialActivationGoal.h
        MocoGoal/MocoInitialVelocityEquilibriumDGFGoal.h
        MocoGoal/MocoInitialVelocityEquilibriumDGFGoal.cpp
        MocoGoal/MocoInitialForceEquilibriumGoal.h
        MocoGoal/MocoInitialForceEquilibriumGoal.cpp
        MocoGoal/MocoPeriodicityGoal.h
        MocoGoal/MocoPeriodicityGoal.cpp
        MocoSolver.h
        MocoSolver.cpp
        MocoDirectCollocationSolver.h
        MocoDirectCollocationSolver.cpp
        MocoTrajectory.h
        MocoTrajectory.cpp
        MocoTropterSolver.h
        MocoTropterSolver.cpp
        MocoParameter.h
        MocoParameter.cpp
        MocoConstraint.h
        MocoConstraint.cpp
        MocoControlBoundConstraint.cpp
        MocoControlBoundConstraint.h
        MocoFrameDistanceConstraint.h
        MocoFrameDistanceConstraint.cpp
        Components/DeGrooteFregly2016Muscle.h
        Components/DeGrooteFregly2016Muscle.cpp
        Components/DiscreteController.cpp
        Components/DiscreteController.h
        Components/StationPlaneContactForce.h
        Components/StationPlaneContactForce.cpp
        Components/PositionMotion.h
        Components/PositionMotion.cpp
        Components/DiscreteForces.h
        Components/DiscreteForces.cpp
        Components/AccelerationMotion.h
        Components/AccelerationMotion.cpp
        Components/MultivariatePolynomialFunction.h
        MocoCasADiSolver/MocoCasADiSolver.h
        MocoCasADiSolver/MocoCasADiSolver.cpp
        MocoInverse.cpp
        MocoInverse.h
        MocoTrack.h
        MocoTrack.cpp
        ModelOperatorsDGF.h
        MocoTool.h
        MocoTool.cpp
        MocoConstraintInfo.h
        MocoConstraintInfo.cpp
        MocoStudyFactory.h
        MocoStudyFactory.cpp
        )
if(OPENSIM_WITH_CASADI)
    list(APPEND MOCO_SOURCES
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
            MocoCasADiSolver/MocoCasOCProblem.h
            MocoCasADiSolver/MocoCasOCProblem.cpp
            )
endif()
if(OPENSIM_WITH_TROPTER)
    list(APPEND MOCO_SOURCES
            tropter/TropterProblem.h
            tropter/TropterProblem.cpp
            )
endif ()
add_library(osimMoco SHARED ${MOCO_SOURCES})
target_link_libraries(osimMoco PUBLIC osimTools)
if (OPENSIM_WITH_CASADI)
    target_link_libraries(osimMoco PRIVATE casadi)
endif()
if (OPENSIM_WITH_TROPTER)
    target_link_libraries(osimMoco PRIVATE tropter)
endif ()

target_include_directories(osimMoco INTERFACE
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

set_target_properties(osimMoco PROPERTIES
        DEFINE_SYMBOL OSIMMOCO_EXPORTS
        PROJECT_LABEL "Library - osimMoco"
        FOLDER "Libraries"
        )
if (OPENSIM_WITH_CASADI)
    target_compile_definitions(osimMoco PUBLIC OPENSIM_WITH_CASADI)
endif ()
if (OPENSIM_WITH_TROPTER)
    target_compile_definitions(osimMoco PUBLIC OPENSIM_WITH_TROPTER)
endif ()
# TODO: Remove the concept of a separate Moco version.
target_compile_definitions(osimMoco PRIVATE
    OPENSIM_MOCO_VERSION=${OPENSIM_MOCO_QUALIFIED_VERSION}
    )

OpenSimAddInstallRPATHSelf(TARGET osimMoco LOADER)
OpenSimAddInstallRPATHSimbody(TARGET osimMoco LOADER
        FROM "${CMAKE_INSTALL_LIBDIR}")

install(TARGETS osimMoco EXPORT OpenSimTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# TODO do not install InverseMuscleSolverMotionData.h b/c it uses Eigen.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/OpenSim/Moco
        FILES_MATCHING PATTERN "*.h*"
        PATTERN "Archive" EXCLUDE
        PATTERN "doc" EXCLUDE
        PATTERN "Test" EXCLUDE
        )





if(BUILD_TESTING)
    add_subdirectory(Test)
endif()
