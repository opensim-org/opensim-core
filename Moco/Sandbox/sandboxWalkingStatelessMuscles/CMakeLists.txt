MocoAddSandboxExecutable(NAME sandboxWalkingStatelessMuscles
        LIB_DEPENDS osimMoco
        RESOURCES
            ../walk_gait1018_state_reference.mot
            "${CMAKE_SOURCE_DIR}/Moco/Tests/testGait10dof18musc_subject01.osim"
        )

