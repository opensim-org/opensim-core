find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

if(OPENSIM_WITH_CASADI)
    set(MOCO_CASADI_TESTS_DISABLED FALSE)
else()
    set(MOCO_CASADI_TESTS_DISABLED TRUE)
endif()

OpenSimAddTest(NAME testMocoInterface
        LINKLIBS osimMoco
        DISABLED ${MOCO_CASADI_TESTS_DISABLED}
        ENVIRONMENT "OPENSIM_MOCO_PARALLEL=0")

OpenSimAddTest(NAME testMocoGoals
        LINKLIBS osimMoco
        DISABLED ${MOCO_CASADI_TESTS_DISABLED}
        ENVIRONMENT "OPENSIM_MOCO_PARALLEL=0")

OpenSimAddTest(NAME testMocoParameters
        LINKLIBS osimMoco
        DISABLED ${MOCO_CASADI_TESTS_DISABLED}
        ENVIRONMENT "OPENSIM_MOCO_PARALLEL=0")

OpenSimAddTest(NAME testMocoImplicit
        LINKLIBS osimMoco
        DISABLED ${MOCO_CASADI_TESTS_DISABLED}
        ENVIRONMENT "OPENSIM_MOCO_PARALLEL=0")

OpenSimAddTest(NAME testMocoConstraints
        LINKLIBS osimMoco
        DISABLED ${MOCO_CASADI_TESTS_DISABLED}
        ENVIRONMENT "OPENSIM_MOCO_PARALLEL=0")

OpenSimAddTest(NAME testMocoContact
        LINKLIBS osimMoco
        DISABLED ${MOCO_CASADI_TESTS_DISABLED}
        ENVIRONMENT "OPENSIM_MOCO_PARALLEL=0"
        RESOURCES resources/subject_20dof18musc_running.osim
                  resources/running_solution_full_stride.sto
        )

OpenSimAddTest(NAME testMocoControllers
        LINKLIBS osimMoco
        DISABLED ${MOCO_CASADI_TESTS_DISABLED}
        ENVIRONMENT "OPENSIM_MOCO_PARALLEL=0")

OpenSimAddTest(NAME testMocoActuators
        LINKLIBS osimMoco
        DISABLED ${MOCO_CASADI_TESTS_DISABLED}
        ENVIRONMENT "OPENSIM_MOCO_PARALLEL=0")

OpenSimAddTest(NAME testMocoInverse
        LINKLIBS osimMoco
        DISABLED ${MOCO_CASADI_TESTS_DISABLED}
        ENVIRONMENT "OPENSIM_MOCO_PARALLEL=0"
        RESOURCES resources/subject_walk_armless_18musc.osim
                  resources/subject_walk_armless_coordinates.mot
                  resources/subject_walk_armless_grfs.mot
                  resources/subject_walk_armless_external_loads.xml
                  resources/std_testMocoInverse_subject_18musc_solution.sto
        )

OpenSimAddTest(NAME testMocoTrack
        LINKLIBS osimMoco
        DISABLED ${MOCO_CASADI_TESTS_DISABLED}
        ENVIRONMENT "OPENSIM_MOCO_PARALLEL=0"
        RESOURCES resources/walk_gait1018_subject01.osim
                  resources/walk_gait1018_state_reference.mot
                  resources/walk_gait1018_subject01_grf.xml
                  resources/walk_gait1018_subject01_grf.mot
                  resources/std_testMocoTrackGait10dof18musc_solution.sto
        )

OpenSimAddTest(NAME testMocoAnalytic
        LINKLIBS osimMoco
        DISABLED ${MOCO_CASADI_TESTS_DISABLED}
        ENVIRONMENT "OPENSIM_MOCO_PARALLEL=0")

OpenSimAddTest(NAME testMocoMetabolics
        LINKLIBS osimMoco
        RESOURCES resources/testMocoMetabolics_hanging_muscle.osim
        )

file(COPY resources/subject_walk_armless_18musc.osim
          resources/subject_walk_armless_coordinates.mot
     DESTINATION "${CMAKE_BINARY_DIR}/OpenSim/Tests/Wrapping")

file(COPY resources/walk_gait1018_subject01.osim
          resources/walk_gait1018_state_reference.mot
     DESTINATION "${CMAKE_BINARY_DIR}/OpenSim/Tests/Wrapping")
