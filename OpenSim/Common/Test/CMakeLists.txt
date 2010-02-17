INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
					${OpenSim_SOURCE_DIR}/Vendors 
					${XERCES_INCLUDE_DIR})

LINK_LIBRARIES( debug osimCommon_d	optimized osimCommon
				debug osimSimulation_d optimized osimSimulation
				debug osimActuators_d optimized osimActuators
				${SIMTK_ALL_LIBS})
IF(WIN32)
    LINK_LIBRARIES( debug xerces-c_2D optimized  xerces-c_2)
ELSE(WIN32)
    LINK_LIBRARIES( debug xerces-c optimized  xerces-c)
ENDIF(WIN32)
				
ADD_EXECUTABLE(testRootSolver testRootSolver.cpp ExampleVectorFunctionUncoupledNxN.cpp)
TARGET_LINK_LIBRARIES(testRootSolver ${LINK_LIBRARIES})
ADD_TEST(testRootSolver ${TEST_PATH}/testRootSolver)

ADD_EXECUTABLE(testSerialization testSerialization.cpp rdSerializableObject.cpp rdSerializableObject2.cpp)
TARGET_LINK_LIBRARIES(testSerialization ${LINK_LIBRARIES})
ADD_TEST(testSerialization ${TEST_PATH}/testSerialization)

ADD_EXECUTABLE(testFunctionAdapter TestFunctionAdapter.cpp)
TARGET_LINK_LIBRARIES(testFunctionAdapter ${LINK_LIBRARIES})
ADD_TEST(testFunctionAdapter ${TEST_PATH}/testFunctionAdapter)

ADD_EXECUTABLE(testGCVSpline TestGCVSpline.cpp)
TARGET_LINK_LIBRARIES(testGCVSpline ${LINK_LIBRARIES})
ADD_TEST(testGCVSpline ${TEST_PATH}/testGCVSpline)


ADD_EXECUTABLE(testStorage testStorage.cpp)
ADD_TEST(testStorage ${TEST_PATH}/testStorage test.sto)
ADD_CUSTOM_COMMAND(
    TARGET testStorage
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${OpenSim_SOURCE_DIR}/OpenSim/Common/Test/test.sto
    ${OpenSim_BINARY_DIR}/OpenSim/Common/Test)
ADD_CUSTOM_COMMAND(
    TARGET testStorage
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${OpenSim_SOURCE_DIR}/OpenSim/Common/Test/testDiff.sto
    ${OpenSim_BINARY_DIR}/OpenSim/Common/Test)

SET_TARGET_PROPERTIES(testRootSolver PROPERTIES PROJECT_LABEL "Tests - testRootSolver")
SET_TARGET_PROPERTIES(testSerialization PROPERTIES PROJECT_LABEL "Tests - testSerialization")
SET_TARGET_PROPERTIES(testFunctionAdapter PROPERTIES PROJECT_LABEL "Tests - testFunctionAdapter")
SET_TARGET_PROPERTIES(testGCVSpline PROPERTIES PROJECT_LABEL "Tests - testGCVSpline")
SET_TARGET_PROPERTIES(testStorage PROPERTIES PROJECT_LABEL "Tests - testStorage")
