add_executable(sandboxSlidingMass EXCLUDE_FROM_ALL
        sandboxSlidingMass.cpp)
set_target_properties(sandboxSlidingMass PROPERTIES
        FOLDER "Muscollo/Sandbox")
target_link_libraries(sandboxSlidingMass osimMuscollo)
