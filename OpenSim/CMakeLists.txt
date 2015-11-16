
subdirs(Common Simulation Actuators Analyses Utilities Tools Examples
    Tests)

add_subdirectory(Sandbox)

install(FILES OpenSim.h DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/OpenSim")
