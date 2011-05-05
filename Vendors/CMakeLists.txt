#-----------------------------------------------------------------------------
IF(BUILD_UTILITIES)
	SUBDIRS(CFSQP MotionAnalysis ${XERCES_DIR_NAME} SimTK)
ELSE(BUILD_UTILITIES)
	SUBDIRS(CFSQP ${XERCES_DIR_NAME} SimTK)
ENDIF(BUILD_UTILITIES)
