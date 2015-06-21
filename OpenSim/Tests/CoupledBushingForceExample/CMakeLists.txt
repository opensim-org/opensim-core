
# Variable definitions
set(EXAMPLE_TARGET exampleCoupledBushingForcePlugin)
set(EXAMPLE_DIR ${OpenSim_SOURCE_DIR}/OpenSim/Examples/Plugins/CoupledBushingForceExample)

include_directories(${OpenSim_SOURCE_DIR} 
            ${OpenSim_SOURCE_DIR}/Vendors)
            
file(GLOB SOURCE_FILES ${EXAMPLE_DIR}/*.h ${EXAMPLE_DIR}/*.cpp)
add_library(${EXAMPLE_TARGET} SHARED ${SOURCE_FILES})

target_link_libraries(${EXAMPLE_TARGET}
    debug osimSimulation${CMAKE_DEBUG_POSTFIX}  optimized osimSimulation
    debug osimActuators${CMAKE_DEBUG_POSTFIX}   optimized osimActuators
    debug osimCommon${CMAKE_DEBUG_POSTFIX}      optimized osimCommon
    debug osimAnalyses${CMAKE_DEBUG_POSTFIX}    optimized osimAnalyses
    debug osimTools${CMAKE_DEBUG_POSTFIX}       optimized osimTools
    ${Simbody_LIBRARIES}
)
set_target_properties(${EXAMPLE_TARGET}
    PROPERTIES
    DEFINE_SYMBOL OSIMPLUGIN_EXPORTS
    PROJECT_LABEL "Examples - ${EXAMPLE_TARGET}"
    RUNTIME_OUTPUT_DIRECTORY ${OpenSim_BINARY_DIR}/OpenSim/Examples/Plugins/CoupledBushingForceExample
)
