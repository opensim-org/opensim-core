
LINK_LIBRARIES( 
        debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
        debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
        debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
        debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
        debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
        ${Simbody_LIBRARIES})

ADD_EXECUTABLE(testComponents testComponents.cpp)

TARGET_LINK_LIBRARIES(testComponents ${LINK_LIBRARIES} )

OpenSimCopySharedTestFiles(gait10dof18musc_subject01.osim)


#
# Testing
#

IF (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
ELSE (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH .)
ENDIF (EXECUTABLE_OUTPUT_PATH)

ADD_TEST(testComponents ${TEST_PATH}/testComponents)

SET_TARGET_PROPERTIES(testComponents PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testComponents")
