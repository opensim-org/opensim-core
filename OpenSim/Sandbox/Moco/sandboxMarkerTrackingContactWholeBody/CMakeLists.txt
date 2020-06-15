MocoAddSandboxExecutable(NAME sandboxMarkerTrackingContactWholeBody
        LIB_DEPENDS osimMoco tropter
        RESOURCES
            gait1018_subject01_onefoot_v30516.osim
            gait1018_subject01_footshank.osim
            ../walk_gait1018_state_reference.mot
            ../walk_gait1018_subject01_grf.mot
        )

MocoAddSandboxExecutable(NAME sandboxCalibrateContact
        LIB_DEPENDS osimTools osimMoco tropter
        RESOURCES
            gait1018_subject01_onefoot_v30516.osim
            gait1018_subject01_onefoot_v30516_opt.osim
            ../walk_gait1018_state_reference.mot
            ../walk_gait1018_subject01_grf.mot
            Rajagopal2016_subject05_v30516.osim
            loadedwalking_subject05_noload_free_trial03_ik_solution.mot
            loadedwalking_subject05_noload_free_trial03_grf.mot
        )
