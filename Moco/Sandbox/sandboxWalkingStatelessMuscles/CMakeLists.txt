MocoAddSandboxExecutable(NAME sandboxWalkingStatelessMuscles
        LIB_DEPENDS osimMoco
        RESOURCES
            ../walk_gait1018_state_reference.mot
            ../walk_gait1018_subject01_grf.mot
            ../walk_gait1018_subject01_grf.xml
            "${CMAKE_SOURCE_DIR}/Moco/Tests/testGait10dof18musc_subject01.osim"
        )

