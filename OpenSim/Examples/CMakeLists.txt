
option(BUILD_API_EXAMPLES
    "Build examples that are used for the APIExamples distribution" ON)

if(BUILD_API_EXAMPLES)

  install(DIRECTORY ExampleMain
        ExampleHopperDevice
        ExampleLuxoMuscle
        ControllerExample
        MuscleExample
        CustomActuatorExample
        OptimizationExample_Arm26
        checkEnvironment
        SimpleOptimizationExample
        SymbolicExpressionReporter
        DataTable
        DataAdapter
        DESTINATION "${OPENSIM_INSTALL_CPPEXDIR}")

    add_subdirectory(ExampleHopperDevice)
    add_subdirectory(SymbolicExpressionReporter)
    add_subdirectory(DataTable)
    add_subdirectory(DataAdapter)
    add_subdirectory(Moco)
    add_subdirectory(Wrapping)

else()

    add_subdirectory(ControllerExample)
    add_subdirectory(ExampleLuxoMuscle)
    add_subdirectory(ExampleMain)
    add_subdirectory(OptimizationExample_Arm26)
    add_subdirectory(CustomActuatorExample)
    add_subdirectory(MuscleExample)
    add_subdirectory(checkEnvironment)
    add_subdirectory(DataAdapter)

endif()

# We do not install the BuildDynamicWalker example, because it is assigned to
# students as homework. The solution to the BuildDynamicWalker example is in
# OpenSim/Examples/BuildDynamicWalker, and is tested in
# OpenSim/Tests/BuildDynamicWalker.

install(DIRECTORY Plugins DESTINATION "${OPENSIM_INSTALL_CPPEXDIR}")
