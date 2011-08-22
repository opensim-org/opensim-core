#-----------------------------------------------------------------------------
IF(BUILD_UTILITIES)
	SUBDIRS(CFSQP lepton MotionAnalysis ${XERCES_DIR_NAME} SimTK)
ELSE(BUILD_UTILITIES)
	SUBDIRS(CFSQP lepton ${XERCES_DIR_NAME} SimTK)
ENDIF(BUILD_UTILITIES)
