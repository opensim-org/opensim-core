MuscolloAddSandboxExecutable(NAME sandboxMarkerTrackingContactWholeBody
        LIB_DEPENDS osimMuscollo tropter
        RESOURCES
            gait1018_subject01_onefoot_v30516.osim
			gait1018_subject01_footshank.osim
            ../walk_marker_trajectories.trc
            ../walk_gait1018_state_reference.mot
            ../walk_gait1018_subject01_grf.mot
        )