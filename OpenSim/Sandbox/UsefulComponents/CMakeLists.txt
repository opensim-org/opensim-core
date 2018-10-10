
file(GLOB COMPONENT_FILES "*.cpp" "*.h")
file(GLOB TO_COMPILE "future*.cpp")

# These executables are *not* tests.

get_filename_component(_target_name ${TO_COMPILE} NAME_WE)
add_executable(${_target_name} EXCLUDE_FROM_ALL ${TO_COMPILE} ${COMPONENT_FILES})
target_link_libraries(${_target_name} osimTools)
set_target_properties(${_target_name} PROPERTIES
    FOLDER "Future sandbox"
)

