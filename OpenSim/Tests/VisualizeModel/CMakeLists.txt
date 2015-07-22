cmake_minimum_required(VERSION 2.6)

set(VISUALIZE_MODEL visualizeModel)

include_directories(${OpenSim_SOURCE_DIR}
                ${OpenSim_SOURCE_DIR}/Vendors)

add_executable(${VISUALIZE_MODEL}
        ${OpenSim_SOURCE_DIR}/OpenSim/Tests/VisualizeModel/VisualizeModel.cpp)

target_link_libraries(${VISUALIZE_MODEL}
       debug osimSimulation${CMAKE_DEBUG_POSTFIX}    optimized osimSimulation
       debug osimActuators${CMAKE_DEBUG_POSTFIX}     optimized osimActuators
       debug osimCommon${CMAKE_DEBUG_POSTFIX}        optimized osimCommon
       debug osimAnalyses${CMAKE_DEBUG_POSTFIX}      optimized osimAnalyses
       debug osimTools${CMAKE_DEBUG_POSTFIX}         optimized osimTools
       ${Simbody_LIBRARIES}
)

set_target_properties(${VISUALIZE_MODEL}
       PROPERTIES
       PROJECT_LABEL "Utilities - ${VISUALIZE_MODEL}"
)

mark_as_advanced(CMAKE_INSTALL_PREFIX)
mark_as_advanced(EXECUTABLE_OUTPUT_PATH)
mark_as_advanced(LIBRARY_OUTPUT_PATH)
