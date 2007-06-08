INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
					${OpenSim_SOURCE_DIR}/Vendors 
					${XERCES_INCLUDE_DIR})

LINK_LIBRARIES( debug xerces-c_2D optimized   xerces-c_2 
				debug osimCommon_d	optimized osimCommon
				debug osimSimulation_d optimized osimSimulation)
				
ADD_EXECUTABLE(testRootSolver testRootSolver.cpp ExampleVectorFunctionUncoupledNxN.cpp)
ADD_TEST(testRootSolver ${TEST_PATH}/testRootSolver)

ADD_EXECUTABLE(testSerialization testSerialization.cpp rdSerializableObject.cpp rdSerializableObject2.cpp)
ADD_TEST(testSerialization ${TEST_PATH}/testSerialization)

SET_TARGET_PROPERTIES(testRootSolver PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testRootSolver")
SET_TARGET_PROPERTIES(testSerialization PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testSerialization")
