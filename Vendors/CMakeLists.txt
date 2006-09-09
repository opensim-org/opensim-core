#-----------------------------------------------------------------------------

IF(WIN32)
SUBDIRS(CFSQP MotionAnalysis xerces-c-src2_4_0)
ELSE(WIN32)
SUBDIRS(CFSQP xerces-c-src2_4_0)
ENDIF(WIN32)
