
subdirs(Common Simulation Actuators Wraps Analyses Utilities Tools Examples
    Tests ExampleComponents)

add_subdirectory(Sandbox)

install(FILES OpenSim.h DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/OpenSim")
