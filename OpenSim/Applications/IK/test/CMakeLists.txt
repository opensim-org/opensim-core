INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors 		  
		    ${XERCES_INCLUDE_DIR})

FILE(GLOB SOURCE_FILES ../Simm*.cpp  ../../../Subject/Simm*.cpp testIK.cpp)
FILE(GLOB INCLUDE_FILES ../Simm*.h ../../../Subject/Simm*.h)

LINK_LIBRARIES( debug xerces-c_2D optimized  xerces-c_2 
		debug osimCommon_d optimized osimCommon
		debug osimSimulation_d optimized osimSimulation)

# SET(EXECUTABLE_OUTPUT_PATH ${OpenSim_SOURCE_DIR}/OpenSim/Applications/IK/test)

ADD_EXECUTABLE(testIK ${SOURCE_FILES} ${INCLUDE_FILES})

SET(TEST_FILES subject.osim subject_ik_setup.xml subject_trial.trc subject_trial.mot std_subject_trial_ik.mot)

#
# Copy files to run dir
#
FOREACH (dataFile ${TEST_FILES})
 ADD_CUSTOM_COMMAND(
    TARGET testIK
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${OpenSim_SOURCE_DIR}/OpenSim/Applications/IK/test/${dataFile}
    ${OpenSim_BINARY_DIR}/OpenSim/Applications/IK/test)
 ENDFOREACH (dataFile) 

ADD_TEST(testIK ${TEST_PATH}/testIK)

