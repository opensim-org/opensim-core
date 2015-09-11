# This folder holds example code that shows what we *want* to be able to do
# with OpenSim in the future. There is an executable target for every
# C++ file whose name begins with `future`, but these executables are not
# built by default (not part of the "all" target).

# These executables are *not* tests.

# If you have pseudocode that you know does *not* compile, you should name
# those as `pseudo*.cpp`.

file(GLOB TO_COMPILE "future*.cpp")

foreach(exec_file ${TO_COMPILE})
    get_filename_component(_target_name ${exec_file} NAME_WE)
    add_executable(${_target_name} EXCLUDE_FROM_ALL ${exec_file})
    target_link_libraries(${_target_name} osimTools)
    set_target_properties(${_target_name} PROPERTIES
        FOLDER "Future sandbox"
        )
endforeach()
