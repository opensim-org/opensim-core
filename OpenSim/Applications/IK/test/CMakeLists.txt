INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors 		  
		    ${OpenSim_SOURCE_DIR}/Vendors/xerces-c-src2_4_0/src)

FILE(GLOB SOURCE_FILES ../simm*.cpp  ../../Scale/simm*.cpp testIK.cpp)
FILE(GLOB INCLUDE_FILES ../simm*.h ../../Scale/simm*.h)

LINK_LIBRARIES( debug xerces-c_2D optimized  xerces-c_2 
		debug rdTools_d optimized rdTools
		debug rdSimulation_d optimized rdSimulation
		debug rdSQP_d optimized rdSQP)

# SET(EXECUTABLE_OUTPUT_PATH ${OpenSim_SOURCE_DIR}/OpenSim/Applications/IK/test)

ADD_DEFINITIONS(-Dworkflow_EXPORTS)

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

