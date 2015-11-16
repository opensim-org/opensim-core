
set(BUILD_API_EXAMPLES ON CACHE BOOL "Build examples that are used for the OpenSimAPIExamples distribution")

if(BUILD_API_EXAMPLES)
## ExampleMain ControllerExample MuscleExample CustomActuatorExample OptimizationExample_Arm26 checkEnvironment
##  install_files(/examples/ FILES README.txt
install(DIRECTORY ExampleMain DESTINATION sdk/APIExamples)
install(DIRECTORY ControllerExample DESTINATION sdk/APIExamples)
install(DIRECTORY MuscleExample DESTINATION sdk/APIExamples)
install(DIRECTORY CustomActuatorExample DESTINATION sdk/APIExamples)
install(DIRECTORY OptimizationExample_Arm26 DESTINATION sdk/APIExamples)
install(DIRECTORY checkEnvironment DESTINATION sdk/APIExamples)
install(DIRECTORY SimpleOptimizationExample DESTINATION sdk/APIExamples)
install(DIRECTORY SymbolicExpressionReporter DESTINATION sdk/APIExamples)
add_subdirectory(SymbolicExpressionReporter)

add_subdirectory(DataTable)
install(DIRECTORY DataTable 
        DESTINATION sdk/APIExamples)

add_subdirectory(DataAdapter)
install(DIRECTORY DataAdapter 
        DESTINATION sdk/APIExamples)

elseif()

add_subdirectory(ControllerExample)
add_subdirectory(ExampleMain)
add_subdirectory(OptimizationExample_Arm26)
add_subdirectory(CustomActuatorExample)
add_subdirectory(MuscleExample)
add_subdirectory(checkEnvironment)

endif()

install(DIRECTORY Plugins DESTINATION  sdk/APIExamples)
