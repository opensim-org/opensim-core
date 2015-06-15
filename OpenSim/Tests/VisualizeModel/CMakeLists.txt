cmake_minimum_required(VERSION 2.6)

SET(VISUALIZE_MODEL visualizeModel)    

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
                ${OpenSim_SOURCE_DIR}/Vendors)

ADD_EXECUTABLE(${VISUALIZE_MODEL} 
        ${OpenSim_SOURCE_DIR}/OpenSim/Tests/VisualizeModel/VisualizeModel.cpp)

TARGET_LINK_LIBRARIES(${VISUALIZE_MODEL}        
       debug osimSimulation${CMAKE_DEBUG_POSTFIX}    optimized osimSimulation
       debug osimActuators${CMAKE_DEBUG_POSTFIX}     optimized osimActuators
       debug osimCommon${CMAKE_DEBUG_POSTFIX}        optimized osimCommon
       debug osimAnalyses${CMAKE_DEBUG_POSTFIX}      optimized osimAnalyses
       debug osimTools${CMAKE_DEBUG_POSTFIX}         optimized osimTools
       ${Simbody_LIBRARIES}
)        

SET_TARGET_PROPERTIES(${VISUALIZE_MODEL}        
       PROPERTIES        
       PROJECT_LABEL "Utilities - ${VISUALIZE_MODEL}"       
)

MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX)
MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH)
MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH)
