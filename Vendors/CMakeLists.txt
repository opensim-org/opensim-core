#-----------------------------------------------------------------------------

IF(OPENSIM_MINIMAL_BUILD)
	SUBDIRS(MotionAnalysis)
ELSE(OPENSIM_MINIMAL_BUILD)
	SUBDIRS(MotionAnalysis ${XERCES_DIR_NAME})
ENDIF(OPENSIM_MINIMAL_BUILD)
