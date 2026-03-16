
add_subdirectory(Common)
add_subdirectory(Simulation)
add_subdirectory(Actuators)
add_subdirectory(Analyses)
add_subdirectory(ExampleComponents)
add_subdirectory(Tools)
add_subdirectory(Moco)
if (NOT OPENSIM_PYTHON_CONDA)
   add_subdirectory(Examples)
   add_subdirectory(Tests)
endif()

install(FILES OpenSim.h DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/OpenSim")
