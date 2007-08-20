SET(UTILNAME openSimToSimm)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
						  ${OpenSim_SOURCE_DIR}/Vendors
						  ${XERCES_INCLUDE_DIR})

# cmake should automatically figure out what other libraries are needed
LINK_LIBRARIES(osimSimmKinematicsEngine)

ADD_EXECUTABLE(${UTILNAME} ${UTILNAME}.cpp)
#INSTALL_TARGETS(/bin/ ${UTILNAME})
SET_TARGET_PROPERTIES(${UTILNAME} PROPERTIES PROJECT_LABEL "Utilities - ${UTILNAME}")
