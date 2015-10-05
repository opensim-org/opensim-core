
file(GLOB TO_COMPILE "future*.cpp")

# These executables are *not* tests.

foreach(exec_file ${TO_COMPILE})
    get_filename_component(_target_name ${exec_file} NAME_WE)
    add_executable(${_target_name} EXCLUDE_FROM_ALL ${exec_file})
    target_link_libraries(${_target_name} osimTools)
    set_target_properties(${_target_name} PROPERTIES
        FOLDER "Future sandbox"
        )
endforeach()
