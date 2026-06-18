
if(BUILD_API_EXAMPLES)
    set(INSTALL_API_EXAMPLES ON CACHE BOOL "" FORCE)
endif()

if(INSTALL_API_EXAMPLES)
  install(DIRECTORY ExampleMain
        ExampleHopperDevice
        ExampleLuxoMuscle
        ExampleSimpleElbow
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
endif()

if(BUILD_API_EXAMPLES)
    add_subdirectory(ExampleHopperDevice)
    add_subdirectory(ExampleSimpleElbow)
    add_subdirectory(SymbolicExpressionReporter)
    add_subdirectory(DataTable)
    add_subdirectory(DataAdapter)
    add_subdirectory(Moco)
    add_subdirectory(Wrapping)
    add_subdirectory(PolynomialPathFitter)
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
