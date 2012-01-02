#-----------------------------------------------------------------------------
IF(BUILD_UTILITIES)
	SUBDIRS(CFSQP lepton MotionAnalysis)
ELSE(BUILD_UTILITIES)
	SUBDIRS(CFSQP lepton)
ENDIF(BUILD_UTILITIES)

IF(WIN32)
	FILE(GLOB VTK_LIBS "vtk_dll/*.dll")
	FOREACH(LIBF ${VTK_LIBS})
		INSTALL(FILES ${LIBF} DESTINATION  bin)
	ENDFOREACH()
ENDIF(WIN32)

