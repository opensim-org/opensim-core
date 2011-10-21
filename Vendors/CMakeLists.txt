#-----------------------------------------------------------------------------
IF(BUILD_UTILITIES)
	SUBDIRS(CFSQP lepton MotionAnalysis ${XERCES_DIR_NAME})
ELSE(BUILD_UTILITIES)
	SUBDIRS(CFSQP lepton ${XERCES_DIR_NAME})
ENDIF(BUILD_UTILITIES)

