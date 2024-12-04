include(CMakeParseArguments)

function(MocoAddSandboxExecutable)
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs LIB_DEPENDS RESOURCES SOURCE_FILES)
    cmake_parse_arguments(MOCOSAND
            "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    add_executable(${MOCOSAND_NAME} EXCLUDE_FROM_ALL ${MOCOSAND_NAME}.cpp 
            ${MOCOSAND_SOURCE_FILES})
    set_target_properties(${MOCOSAND_NAME} PROPERTIES
            FOLDER "Sandbox/Moco")
    target_link_libraries(${MOCOSAND_NAME} ${MOCOSAND_LIB_DEPENDS})
    target_include_directories(${MOCOSAND_NAME} PRIVATE
            ${CMAKE_SOURCE_DIR}/Sandbox/Moco/shared)
    file(COPY ${MOCOSAND_RESOURCES} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
endfunction()

MocoAddSandboxExecutable(NAME sandboxSandbox LIB_DEPENDS osimMoco)

MocoAddSandboxExecutable(NAME sandboxTendonForceState
        LIB_DEPENDS osimMoco tropter)
MocoAddSandboxExecutable(NAME sandboxImplicitActivationDynamics
        LIB_DEPENDS osimMoco tropter)


# MocoStudy-related.
MocoAddSandboxExecutable(NAME sandboxSlidingMass
        LIB_DEPENDS osimMoco tropter)
MocoAddSandboxExecutable(NAME sandboxContact
        LIB_DEPENDS osimMoco osimAnalyses tropter)
MocoAddSandboxExecutable(NAME sandboxMuscle
        LIB_DEPENDS osimMoco)
MocoAddSandboxExecutable(NAME sandboxCasADi
        LIB_DEPENDS osimMoco casadi tropter)

MocoAddSandboxExecutable(NAME sandboxCasADiParallelMap
        LIB_DEPENDS SimTKcommon casadi)

MocoAddSandboxExecutable(NAME sandboxSimTKMotion
        LIB_DEPENDS SimTKsimbody)

add_subdirectory(sandboxWholeBodyTracking)
add_subdirectory(sandboxMarkerTrackingWholeBody)
add_subdirectory(sandboxJointReaction)
add_subdirectory(sandboxOneLegCouplerKnee)
add_subdirectory(sandboxMarkerTrackingContactWholeBody)
add_subdirectory(sandboxSquatToStand)
add_subdirectory(sandboxMocoTrack)
add_subdirectory(sandboxMocoUserControlCost)
