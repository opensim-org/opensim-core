find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

set(OSIM_WRAP_TEST_LIBS osimCommon osimSimulation osimActuators)

OpenSimAddTest(NAME testGeometryPathWrapping
        LINKLIBS ${OSIM_WRAP_TEST_LIBS}
        RESOURCES resources/std_testGeometryPathWrapping_subject_walk_armless_muscle_lengths.sto
                  resources/std_testGeometryPathWrapping_walk_gait1018_muscle_lengths.sto
                  resources/testGeometryPathWrapping_ShoulderWrappingWithPathSprings.osim
                  "${OPENSIM_TEST_RESOURCES_DIR}/subject_walk_armless_18musc.osim"
                  "${OPENSIM_TEST_RESOURCES_DIR}/subject_walk_armless_coordinates.mot"
                  "${OPENSIM_TEST_RESOURCES_DIR}/walk_gait1018_subject01.osim"
                  "${OPENSIM_TEST_RESOURCES_DIR}/walk_gait1018_state_reference.mot"
        )

OpenSimAddTest(NAME testScholz2015GeometryPath
        LINKLIBS ${OSIM_WRAP_TEST_LIBS})

OpenSimAddTest(NAME testWrapCylinder
        LINKLIBS ${OSIM_WRAP_TEST_LIBS})

OpenSimAddTest(NAME testWrapObject
        LINKLIBS ${OSIM_WRAP_TEST_LIBS}
        RESOURCES resources/testWrapObject_updateFromXMLNode30515.osim
        )
