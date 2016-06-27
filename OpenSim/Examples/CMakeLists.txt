
option(BUILD_API_EXAMPLES
    "Build examples that are used for the APIExamples distribution" ON)

if(BUILD_API_EXAMPLES)

    install(DIRECTORY ExampleMain
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

    add_subdirectory(SymbolicExpressionReporter)
    add_subdirectory(DataTable)

elseif()

    add_subdirectory(ControllerExample)
    add_subdirectory(ExampleLuxoMuscle)
    add_subdirectory(ExampleMain)
    add_subdirectory(OptimizationExample_Arm26)
    add_subdirectory(CustomActuatorExample)
    add_subdirectory(MuscleExample)
    add_subdirectory(checkEnvironment)

endif()

install(DIRECTORY Plugins DESTINATION "${OPENSIM_INSTALL_CPPEXDIR}")
