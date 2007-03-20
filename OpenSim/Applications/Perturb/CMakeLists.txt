INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
						  ${OpenSim_SOURCE_DIR}/Vendors
						  ${XERCES_INCLUDE_DIR})

# cmake should automatically figure out what other libraries are needed
LINK_LIBRARIES(debug osimTools_d optimized osimTools)

ADD_EXECUTABLE(perturb perturb.cpp)

INSTALL_TARGETS(/bin/ perturb)
