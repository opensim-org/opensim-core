SET(APPNAME cmc)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
						  ${OpenSim_SOURCE_DIR}/Vendors
						  ${XERCES_INCLUDE_DIR})

# cmake should automatically figure out what other libraries are needed
LINK_LIBRARIES(debug osimTools_d optimized osimTools)

ADD_EXECUTABLE(${APPNAME} ${APPNAME}.cpp)
INSTALL_TARGETS(/bin/ ${APPNAME})
SET_TARGET_PROPERTIES(${APPNAME} PROPERTIES PROJECT_LABEL "Applications - ${APPNAME}")

#
# Testing
#

#IF (UNIX)
#  ADD_DEFINITIONS(-fprofile-arcs -ftest-coverage)
#  LINK_LIBRARIES(gcov)
#ENDIF (UNIX)
