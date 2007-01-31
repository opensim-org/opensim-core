INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors ${XERCES_INCLUDE_DIR})

FILE(GLOB SOURCE_FILES *.cpp ../../Subject/Simm*.cpp ../../Applications/IK/SimmInverseKinematicsTarget.cpp ../../Applications/IK/SimmIKSolverImpl.cpp ../../Applications/IK/IKTool.cpp)
FILE(GLOB INCLUDE_FILES *.h ../../Subject/Simm*.h ../../Applications/IK/SimmInverseKinematicsTarget.h ../../Applications/IK/SimmIKSolverImpl.h ../../Applications/IK/IKTool.h)

ADD_DEFINITIONS(-Dworkflow_EXPORTS -DRDSIMULATION_EXPORTS)

LINK_LIBRARIES(debug xerces-c_2D optimized xerces-c_2 
	debug rdTools_d optimized rdTools 
	debug rdSimulation_d optimized rdSimulation
	debug rdCMC_d optimized rdCMC
	debug suAnalyses_d optimized suAnalyses
	debug rdActuators_d optimized rdActuators
	debug rdSQP_d optimized rdSQP)

ADD_EXECUTABLE(addComments ${SOURCE_FILES} ${INCLUDE_FILES})

INSTALL_TARGETS(/bin/ addComments)
