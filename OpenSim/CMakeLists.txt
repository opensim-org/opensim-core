
add_subdirectory(Common)
add_subdirectory(Simulation)
add_subdirectory(Actuators)
add_subdirectory(Analyses)
add_subdirectory(Utilities)
add_subdirectory(ExampleComponents)
add_subdirectory(Tools)
add_subdirectory(Moco)
add_subdirectory(Examples)
add_subdirectory(Tests)

#add_subdirectory(Sandbox)

install(FILES OpenSim.h DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/OpenSim")
