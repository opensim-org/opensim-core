include(CMakeParseArguments)

function(MuscolloAddSandboxExecutable)
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs LIB_DEPENDS RESOURCES)
    cmake_parse_arguments(MUCOSAND
            "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    add_executable(${MUCOSAND_NAME} EXCLUDE_FROM_ALL ${MUCOSAND_NAME}.cpp)
    set_target_properties(${MUCOSAND_NAME} PROPERTIES
            FOLDER "Muscollo/Sandbox")
    target_link_libraries(${MUCOSAND_NAME} ${MUCOSAND_LIB_DEPENDS})
    file(COPY ${MUCOSAND_RESOURCES} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
endfunction()


MuscolloAddSandboxExecutable(NAME sandboxSlidingMass
        LIB_DEPENDS osimMuscollo tropter)
MuscolloAddSandboxExecutable(NAME sandboxContact
        LIB_DEPENDS osimMuscollo osimAnalyses tropter)
MuscolloAddSandboxExecutable(NAME sandboxDoublePendulumSwingup
        LIB_DEPENDS osimMuscollo tropter)
MuscolloAddSandboxExecutable(NAME sandboxTendonForceState
        LIB_DEPENDS osimMuscollo tropter)
MuscolloAddSandboxExecutable(NAME sandboxImplicitActivationDynamics
        LIB_DEPENDS osimMuscollo tropter)
		
add_subdirectory(sandboxWholeBodyTracking)
add_subdirectory(sandboxMarkerTrackingWholeBody)
