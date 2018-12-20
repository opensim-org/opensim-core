include(CMakeParseArguments)

function(MocoAddSandboxExecutable)
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs LIB_DEPENDS RESOURCES SOURCE_FILES)
    cmake_parse_arguments(MUCOSAND
            "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    add_executable(${MUCOSAND_NAME} EXCLUDE_FROM_ALL ${MUCOSAND_NAME}.cpp 
            ${MUCOSAND_SOURCE_FILES})
    set_target_properties(${MUCOSAND_NAME} PROPERTIES
            FOLDER "Moco/Sandbox")
    target_link_libraries(${MUCOSAND_NAME} ${MUCOSAND_LIB_DEPENDS})
    target_include_directories(${MUCOSAND_NAME} PRIVATE
            ${CMAKE_SOURCE_DIR}/Moco/Sandbox/shared)
    file(COPY ${MUCOSAND_RESOURCES} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
endfunction()


MocoAddSandboxExecutable(NAME sandboxTendonForceState
        LIB_DEPENDS osimMoco tropter)
MocoAddSandboxExecutable(NAME sandboxImplicitActivationDynamics
        LIB_DEPENDS osimMoco tropter)


# MocoTool-related.
MocoAddSandboxExecutable(NAME sandboxSlidingMass
        LIB_DEPENDS osimMoco tropter)
MocoAddSandboxExecutable(NAME sandboxContact
        LIB_DEPENDS osimMoco osimAnalyses tropter)
MocoAddSandboxExecutable(NAME sandboxMuscle
        SOURCE_FILES
            shared/DeGrooteFregly2016Muscle.h
            shared/DeGrooteFregly2016Muscle.cpp
        LIB_DEPENDS osimMoco)

add_subdirectory(sandboxWholeBodyTracking)
add_subdirectory(sandboxMarkerTrackingWholeBody)
add_subdirectory(sandboxJointReaction)
add_subdirectory(sandboxOneLegCouplerKnee)
add_subdirectory(sandboxMarkerTrackingContactWholeBody)
