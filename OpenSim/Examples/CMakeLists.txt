
set(BUILD_API_EXAMPLES ON CACHE BOOL "Build examples that are used for the OpenSimAPIExamples distribution")
mark_as_advanced(BUILD_API_EXAMPLES)

IF (BUILD_API_EXAMPLES)
## ExampleMain ControllerExample MuscleExample CustomActuatorExample OptimizationExample_Arm26 checkEnvironment
##  install_files(/examples/ FILES README.txt
install(DIRECTORY ExampleMain DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
install(DIRECTORY ControllerExample DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
install(DIRECTORY MuscleExample DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
install(DIRECTORY CustomActuatorExample DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
install(DIRECTORY OptimizationExample_Arm26 DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
install(DIRECTORY checkEnvironment DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
install(DIRECTORY SimpleOptimizationExample DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
install(DIRECTORY SymbolicExpressionReporter DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
add_subdirectory(SymbolicExpressionReporter)

ELSEIF (BUILD_API_EXAMPLES)
add_subdirectory(ControllerExample)
add_subdirectory(ExampleMain)
add_subdirectory(OptimizationExample_Arm26)
add_subdirectory(CustomActuatorExample)
add_subdirectory(MuscleExample)
add_subdirectory(checkEnvironment)

ENDIF (BUILD_API_EXAMPLES)

install(DIRECTORY Plugins DESTINATION  sdk/APIExamples  PATTERN ".svn" EXCLUDE)
