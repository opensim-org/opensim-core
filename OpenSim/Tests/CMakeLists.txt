include_directories(${OpenSim_SOURCE_DIR}
                    ${OpenSim_SOURCE_DIR}/Vendors)

if(BUILD_TESTING)
    add_subdirectory(Components)
    add_subdirectory(ControllerExample)
    add_subdirectory(CoupledBushingForceExample)
    add_subdirectory(ExampleMain)
    add_subdirectory(OptimizationExample_Arm26)
    add_subdirectory(CustomActuatorExample)
    add_subdirectory(MuscleExample)
    add_subdirectory(SimpleOptimizationExample)
    add_subdirectory(Environment)
    add_subdirectory(testIterators)
    add_subdirectory(VisualizeModel)
    add_subdirectory(README)
    add_subdirectory(Wrapping)
endif()

add_subdirectory(Future)

