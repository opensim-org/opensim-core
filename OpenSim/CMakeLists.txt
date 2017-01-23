
subdirs(Common Simulation Actuators Analyses Utilities Tools Examples
    Tests)

add_dependencies(osimCommon     Simbody_CONFIG_check)
add_dependencies(osimSimulation Simbody_CONFIG_check)
add_dependencies(osimActuators  Simbody_CONFIG_check)
add_dependencies(osimAnalyses   Simbody_CONFIG_check)
add_dependencies(osimTools      Simbody_CONFIG_check)

add_subdirectory(Sandbox)

install(FILES OpenSim.h DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/OpenSim")
