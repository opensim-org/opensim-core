SET(APPNAME scale)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
						  ${OpenSim_SOURCE_DIR}/Vendors
						  ${XERCES_INCLUDE_DIR})
						  
ADD_EXECUTABLE(${APPNAME} ${APPNAME}.cpp)

# cmake should automatically figure out what other libraries are needed
TARGET_LINK_LIBRARIES(${APPNAME}
          debug osimTools_d optimized osimTools
          debug osimCommon_d optimized osimCommon
          debug osimSimulation_d optimized osimSimulation
          debug ${NameSpace}SimTKcommon_d optimized ${NameSpace}SimTKcommon)


INSTALL_TARGETS(/bin/ ${APPNAME})
SET_TARGET_PROPERTIES(${APPNAME} PROPERTIES PROJECT_LABEL "Applications - ${APPNAME}")

#
# Testing
#

#IF (UNIX)
#  ADD_DEFINITIONS(-fprofile-arcs -ftest-coverage)
#  LINK_LIBRARIES(gcov)
#ENDIF (UNIX)

#ENABLE_TESTING()

IF (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
ELSE (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH .)
ENDIF (EXECUTABLE_OUTPUT_PATH)

SUBDIRS(test)
