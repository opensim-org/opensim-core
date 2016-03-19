
# Variable definitions
set(LUXO_DIR ${OpenSim_SOURCE_DIR}/OpenSim/Examples/ExampleLuxoMuscle)

file(GLOB SOURCE_FILES ${LUXO_DIR}/*.h ${LUXO_DIR}/*.cpp)
add_executable(exampleLuxoMuscle ${SOURCE_FILES})
target_link_libraries(exampleLuxoMuscle osimTools)
set_target_properties(exampleLuxoMuscle PROPERTIES
    PROJECT_LABEL "Luxo Muscle"
    FOLDER "Examples"
    EXCLUDE_FROM_ALL TRUE
    )

add_executable(exampleHopperDevice exampleHopperDevice.cpp answers.cpp)
target_link_libraries(exampleHopperDevice osimTools)
set_target_properties(exampleHopperDevice PROPERTIES
    PROJECT_LABEL "Hopper Device"
    FOLDER "Examples"
    EXCLUDE_FROM_ALL TRUE
    )

file(GLOB RESOURCE_FILES
    ${LUXO_DIR}/*.obj 
    ${CMAKE_CURRENT_SOURCE_DIR}/*.osim
    ${CMAKE_CURRENT_SOURCE_DIR}/*.vtp
    )

foreach(dataFile ${RESOURCE_FILES})
    file(COPY ${dataFile} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endforeach()


# Create a custom target 
if(MSVC OR XCODE) 
    #    add_custom_command(
    #        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/Luxo_Myo.osim"
    #        COMMAND "${CMAKE_BINARY_DIR}/${CMAKE_CFG_INTDIR}/exampleLuxoMuscle"
    #        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    #        DEPENDS exampleLuxoMuscle
    #        )
    add_custom_target(RunExampleHopperDevice
        COMMAND
            "${CMAKE_BINARY_DIR}/${CMAKE_CFG_INTDIR}/exampleHopperDevice"
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        DEPENDS exampleHopperDevice
        #        "${CMAKE_CURRENT_BINARY_DIR}/Luxo_Myo.osim"
        )
else()
    #    add_custom_command(
    #        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/Luxo_Myo.osim"
    #        COMMAND "${CMAKE_BINARY_DIR}/exampleLuxoMuscle"
    #        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    #        DEPENDS exampleLuxoMuscle
    #        )
    add_custom_target(RunExampleHopperDevice
        COMMAND "${CMAKE_BINARY_DIR}/exampleHopperDevice"
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        DEPENDS exampleHopperDevice
        #"${CMAKE_CURRENT_BINARY_DIR}/Luxo_Myo.osim"
        )
endif()

set_target_properties(RunExampleHopperDevice PROPERTIES
    PROJECT_LABEL "Run Hopper Device")
