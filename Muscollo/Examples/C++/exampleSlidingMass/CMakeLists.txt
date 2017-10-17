
add_executable(exampleSlidingMass exampleSlidingMass.cpp)
set_target_properties(exampleSlidingMass PROPERTIES
        FOLDER "Muscollo")
target_link_libraries(exampleSlidingMass osimMuscollo)
