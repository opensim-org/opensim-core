
SET(BUILD_API_EXAMPLES ON CACHE BOOL "Build examples that are used for the OpenSimAPIExamples distribution")
MARK_AS_ADVANCED(BUILD_API_EXAMPLES)

IF (BUILD_API_EXAMPLES)
## ExampleMain ControllerExample MuscleExample CustomActuatorExample OptimizationExample_Arm26 checkEnvironment
##  INSTALL_FILES(/examples/ FILES README.txt
INSTALL(DIRECTORY ExampleMain DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
INSTALL(DIRECTORY ControllerExample DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
INSTALL(DIRECTORY MuscleExample DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
INSTALL(DIRECTORY CustomActuatorExample DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
INSTALL(DIRECTORY OptimizationExample_Arm26 DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
INSTALL(DIRECTORY checkEnvironment DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
INSTALL(DIRECTORY SimpleOptimizationExample DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
INSTALL(DIRECTORY SymbolicExpressionReporter DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
ADD_SUBDIRECTORY(SymbolicExpressionReporter)

ELSEIF (BUILD_API_EXAMPLES)
ADD_SUBDIRECTORY(ControllerExample)
ADD_SUBDIRECTORY(ExampleMain)
ADD_SUBDIRECTORY(OptimizationExample_Arm26)
ADD_SUBDIRECTORY(CustomActuatorExample)
ADD_SUBDIRECTORY(MuscleExample)
ADD_SUBDIRECTORY(checkEnvironment)

ENDIF (BUILD_API_EXAMPLES)

INSTALL(DIRECTORY Plugins DESTINATION  sdk/APIExamples  PATTERN ".svn" EXCLUDE)

if(BUILD_API_EXAMPLES)
  add_subdirectory(DataTable)
  install(DIRECTORY   DataTable
          DESTINATION sdk/APIExamples)

  add_subdirectory(DataAdapter)
  install(DIRECTORY   DataAdapter
          DESTINATION sdk/APIExamples)
endif(BUILD_API_EXAMPLES)
