add_executable(sandboxWholeBodyTracking EXCLUDE_FROM_ALL
        sandboxWholeBodyTracking.cpp)
set_target_properties(sandboxWholeBodyTracking PROPERTIES
        FOLDER "Muscollo/Sandbox/sandboxWholeBodyTracking")
target_link_libraries(sandboxWholeBodyTracking osimMuscollo tropter)