find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

OpenSimAddTest(NAME testControllers
        LINKLIBS osimTools
        RESOURCES resources/arm26.osim
                  resources/arm26_Reserve_Actuators.xml
                  resources/arm26_controls.xml
                  resources/block.vtp
                  resources/testControllers_TugOfWar.osim
        )

OpenSimAddTest(NAME testExternalLoads
        LINKLIBS osimTools
        RESOURCES resources/Pendulum.osim
                  resources/cylinder.vtp
        )

OpenSimAddTest(NAME testModelCopy
        LINKLIBS osimTools
        RESOURCES resources/Neck3dof_point_constraint.osim
                  resources/arm26.osim
        )

OpenSimAddTest(NAME testSerializeOpenSimObjects
        LINKLIBS osimTools
        RESOURCES resources/ikToolNoDocumentTag.xml
        )

OpenSimAddTest(NAME testVisualization
        LINKLIBS osimTools
        RESOURCES resources/BuiltinGeometry.osim
                  resources/block.vtp
                  resources/cylinder.vtp
                  resources/double_pendulum33.osim
                  resources/doubletransform33.osim
                  resources/sphere.vtp
                  resources/test_wrapAllVis.osim
                  resources/vis_AppearanceTest.txt
                  resources/vis_BuiltinGeometry.txt
                  resources/visualize_contacts.osim
        )

# edge-case: one test ensures that `ExternalLoads` still works - even when it isn't in
#            the working directory of the test runner (#3926)
file(COPY "resources/ExternalLoadsInSubdir" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
