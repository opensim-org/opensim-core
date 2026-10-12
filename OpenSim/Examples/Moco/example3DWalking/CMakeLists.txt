set(EXAMPLE_MOCO_TRACK_FILES
        subject_walk_scaled.osim
        subject_walk_scaled_FunctionBasedPathSet.xml
        subject_walk_scaled_ContactForceSet.xml
        subject_walk_scaled_ContactGeometrySet.xml
        subject_walk_scaled_ExpressionBasedCoordinateForceSet.xml
        coordinates.sto
        grf_walk.mot
        grf_walk.xml
        markers_walk.trc
        ik_tasks_walk.xml
        electromyography.sto
        id_walk.sto
        README.txt)

OpenSimAddExampleCXX(NAME example3DWalking SUBDIR Moco
        EXECUTABLES exampleMocoTrack exampleMocoInverse example3DWalking
        RESOURCES "${EXAMPLE_MOCO_TRACK_FILES}")
