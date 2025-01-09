
file(GLOB TASK_SPACE_FILES "*.cpp" "*.h")
file(GLOB TO_COMPILE "future*.cpp")

# These executables are *not* tests.

set(SANDBOX_FILES
    "futureTaskSpace.osim"
)

foreach(dataFile ${SANDBOX_FILES})
    configure_file(${dataFile} ${CMAKE_CURRENT_BINARY_DIR}/${dataFile} COPYONLY)
endforeach()

get_filename_component(_target_name ${TO_COMPILE} NAME_WE)
add_executable(${_target_name} EXCLUDE_FROM_ALL ${TO_COMPILE} ${TASK_SPACE_FILES})
target_link_libraries(${_target_name} osimTools)
set_target_properties(${_target_name} PROPERTIES
    FOLDER "Future sandbox"
)

