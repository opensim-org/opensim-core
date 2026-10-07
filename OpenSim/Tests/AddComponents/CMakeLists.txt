find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

OpenSimAddTest(NAME testAddComponents
        LINKLIBS osimTools osimSimulation osimActuators
        RESOURCES "${OpenSim_SOURCE_DIR}/OpenSim/Examples/ExampleMain/blockMesh.obj"
                  "${OpenSim_SOURCE_DIR}/OpenSim/Tests/ExampleMain/std_tugOfWar_forces.mot"
                  "${OpenSim_SOURCE_DIR}/OpenSim/Tests/ExampleMain/std_tugOfWar_states.sto"
        )
