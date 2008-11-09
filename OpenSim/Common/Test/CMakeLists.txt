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

ADD_EXECUTABLE(testFunctionAdapter testFunctionAdapter.cpp)
ADD_TEST(testFunctionAdapter ${TEST_PATH}/testFunctionAdapter)

ADD_EXECUTABLE(testGCVSpline testGCVSpline.cpp)
ADD_TEST(testGCVSpline ${TEST_PATH}/testGCVSpline)

ADD_EXECUTABLE(testRegistration testRegistration.cpp)
ADD_TEST(testRegistration ${TEST_PATH}/testRegistration)


ADD_EXECUTABLE(testStorage testStorage.cpp)
ADD_TEST(testStorage ${TEST_PATH}/testStorage)
ADD_CUSTOM_COMMAND(
    TARGET testStorage
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${OpenSim_SOURCE_DIR}/OpenSim/Common/Test/test.sto
    ${OpenSim_BINARY_DIR}/OpenSim/Common/Test)

SET_TARGET_PROPERTIES(testRootSolver PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testRootSolver")
SET_TARGET_PROPERTIES(testSerialization PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testSerialization")
SET_TARGET_PROPERTIES(testFunctionAdapter PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testFunctionAdapter")
SET_TARGET_PROPERTIES(testGCVSpline PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testGCVSpline")
SET_TARGET_PROPERTIES(testStorage PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testStorage")
SET_TARGET_PROPERTIES(testRegistration PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testRegistration")
