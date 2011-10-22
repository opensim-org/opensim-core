#-----------------------------------------------------------------------------
IF(BUILD_UTILITIES)
	SUBDIRS(CFSQP lepton MotionAnalysis ${XERCES_DIR_NAME})
ELSE(BUILD_UTILITIES)
	SUBDIRS(CFSQP lepton ${XERCES_DIR_NAME})
ENDIF(BUILD_UTILITIES)

IF(WIN32)
	FILE(GLOB VTK_LIBS "vtk_dll/*.dll")
	FOREACH(LIBF ${VTK_LIBS})
		INSTALL(FILES ${LIBF} DESTINATION  bin)
	ENDFOREACH()
ENDIF(WIN32)

