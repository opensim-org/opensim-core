INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
					${OpenSim_SOURCE_DIR}/Vendors 
					${XERCES_INCLUDE_DIR})

LINK_LIBRARIES( debug xerces-c_2D optimized   xerces-c_2 
				debug osimCommon_d	optimized osimCommon
				debug osimSimulation_d optimized osimSimulation)
				
ADD_EXECUTABLE(testRootSolver testRootSolver.cpp ExampleVectorFunctionUncoupledNxN.cpp)
TARGET_LINK_LIBRARIES(testRootSolver ${LINK_LIBRARIES})
ADD_TEST(testRootSolver ${TEST_PATH}/testRootSolver)

ADD_EXECUTABLE(testSerialization testSerialization.cpp rdSerializableObject.cpp rdSerializableObject2.cpp)
TARGET_LINK_LIBRARIES(testSerialization ${LINK_LIBRARIES})
ADD_TEST(testSerialization ${TEST_PATH}/testSerialization)

ADD_EXECUTABLE(testFunctionAdapter testFunctionAdapter.cpp)
TARGET_LINK_LIBRARIES(testFunctionAdapter osimCommon xerces-c_2 simtkCommon simtkCpodes simtkLapack simtkMath)
ADD_TEST(testFunctionAdapter ${TEST_PATH}/testFunctionAdapter)

ADD_EXECUTABLE(testGCVSpline testGCVSpline.cpp)
TARGET_LINK_LIBRARIES(testGCVSpline ${LINK_LIBRARIES})
ADD_TEST(testGCVSpline ${TEST_PATH}/testGCVSpline)

SET_TARGET_PROPERTIES(testRootSolver PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testRootSolver")
SET_TARGET_PROPERTIES(testSerialization PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testSerialization")
SET_TARGET_PROPERTIES(testFunctionAdapter PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testFunctionAdapter")
SET_TARGET_PROPERTIES(testGCVSpline PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testGCVSpline")
