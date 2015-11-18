
file(GLOB TO_COMPILE "future*.cpp")
file(GLOB TASK_SPACE_FILES "TaskSpace/*.cpp" "TaskSpace/*.h")

# These executables are *not* tests.

include_directories(${OpenSim_SOURCE_DIR}
                    ${OpenSim_SOURCE_DIR}/Vendors)

set(SANDBOX_FILES 
    "futureOrientationInverseKinematics.osim"
    "futureOrientationInverseKinematics.trc"
	"futureTaskSpace.osim"
)

foreach(dataFile ${SANDBOX_FILES})
    configure_file(${dataFile} ${CMAKE_CURRENT_BINARY_DIR}/${dataFile} COPYONLY)
endforeach()

foreach(exec_file ${TO_COMPILE})
    get_filename_component(_target_name ${exec_file} NAME_WE)
    add_executable(${_target_name} EXCLUDE_FROM_ALL ${exec_file})
	if(${exec_file} MATCHES "futureTaskSpace.cpp")
		add_library(taskSpaceLib STATIC ${TASK_SPACE_FILES})
		target_link_libraries(${_target_name} taskSpaceLib)
	endif()
    target_link_libraries(${_target_name} osimTools)
    set_target_properties(${_target_name} PROPERTIES
        FOLDER "Future sandbox"
        )
endforeach()
