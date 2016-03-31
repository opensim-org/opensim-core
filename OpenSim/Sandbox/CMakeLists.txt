add_subdirectory(TaskSpace EXCLUDE_FROM_ALL)

file(GLOB TO_COMPILE "future*.cpp")

# These executables are *not* tests.

include_directories(${OpenSim_SOURCE_DIR}
                    ${OpenSim_SOURCE_DIR}/Vendors)

add_subdirectory(ExampleHopperDevice)


set(SANDBOX_FILES
    "futureOrientationInverseKinematics.osim"
    "futureOrientationInverseKinematics.trc"
)

foreach(dataFile ${SANDBOX_FILES})
    configure_file(${dataFile} ${CMAKE_CURRENT_BINARY_DIR}/${dataFile} COPYONLY)
endforeach()

OpenSimCopySharedTestFiles(gait10dof18musc_subject01.osim)

foreach(exec_file ${TO_COMPILE})
    get_filename_component(_target_name ${exec_file} NAME_WE)
    add_executable(${_target_name} EXCLUDE_FROM_ALL ${exec_file})
    target_link_libraries(${_target_name} osimTools)
    set_target_properties(${_target_name} PROPERTIES
        FOLDER "Future sandbox"
    )
endforeach()


if(UNIX)
    add_executable(ImuStreaming ImuStreaming.cpp)
    target_link_libraries(ImuStreaming osimCommon osimSimulation osimTools)
endif()