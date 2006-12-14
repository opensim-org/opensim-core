INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors 		    
		    ${XERCES_INCLUDE_DIR})


LINK_LIBRARIES( debug xerces-c_2D optimized  xerces-c_2 
		debug rdTools_d optimized rdTools
		debug rdSimulation_d optimized rdSimulation
		debug rdActuators_d optimized rdActuators
		debug suAnalyses_d optimized suAnalyses)
ADD_EXECUTABLE(perturb perturb.cpp)

INSTALL_TARGETS(/bin/ perturb)
