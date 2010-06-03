SUBDIRS(Gait2354_Simbody Gait2392_Simbody Tutorials Arm26)
# Don't install README.txt for now... it's out of date
#INSTALL_FILES(/examples/ FILES README.txt)
SET(BUILD_APIEXAMPLES ON CACHE BOOL "Build examples that are used for the OpenSimAPIExamples distribution")
MARK_AS_ADVANCED(BUILD_APIEXAMPLES)

IF (BUILD_APIEXAMPLES)
## ExampleMain ControllerExample MuscleExample CustomActuatorExample OptimizationExample_Arm26 TestEnvironment
##	INSTALL_FILES(/examples/ FILES README.txt
INSTALL(DIRECTORY ExampleMain DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
INSTALL(DIRECTORY ControllerExample DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
INSTALL(DIRECTORY MuscleExample DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
INSTALL(DIRECTORY CustomActuatorExample DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
INSTALL(DIRECTORY OptimizationExample_Arm26 DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
INSTALL(DIRECTORY testEnvironment DESTINATION sdk/APIExamples PATTERN ".svn" EXCLUDE)
ENDIF (BUILD_APIEXAMPLES)
