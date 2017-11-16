add_executable(sandboxSlidingMass EXCLUDE_FROM_ALL
        sandboxSlidingMass.cpp)
set_target_properties(sandboxSlidingMass PROPERTIES
        FOLDER "Muscollo/Sandbox")
target_link_libraries(sandboxSlidingMass osimMuscollo tropter)

add_executable(sandboxTendonForceState EXCLUDE_FROM_ALL
        sandboxTendonForceState.cpp)
set_target_properties(sandboxTendonForceState PROPERTIES
        FOLDER "Muscollo/Sandbox")
target_link_libraries(sandboxTendonForceState osimMuscollo tropter)

add_executable(sandboxImplicitActivationDynamics EXCLUDE_FROM_ALL
        sandboxImplicitActivationDynamics.cpp)
set_target_properties(sandboxImplicitActivationDynamics PROPERTIES
        FOLDER "Muscollo/Sandbox")
target_link_libraries(sandboxImplicitActivationDynamics osimMuscollo tropter)

add_subdirectory(sandboxWholeBodyTracking)
