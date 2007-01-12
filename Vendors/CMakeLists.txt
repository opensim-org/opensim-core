#-----------------------------------------------------------------------------

IF(OPENSIM_MINIMAL_BUILD)
	SUBDIRS(CFSQP MotionAnalysis)
ELSE(OPENSIM_MINIMAL_BUILD)
	SUBDIRS(CFSQP MotionAnalysis ${XERCES_DIR_NAME})
ENDIF(OPENSIM_MINIMAL_BUILD)
