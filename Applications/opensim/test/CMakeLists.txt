
OpenSimAddTests(
    TESTPROGRAMS testCommandLineInterface
    # Don't need OpenSim since we only interact with OpenSim through the
    # command line tool. But we use Simbody for its Testing.h.
    LINKLIBS SimTKsimbody
    )

add_dependencies(testCommandLineInterface opensim-cli)

if(${CMAKE_VERSION} VERSION_LESS 3.0)
    get_target_property(osim_cli_path opensim-cli LOCATION)
    get_target_property(osim_actuators_lib_path osimActuators LOCATION)
    set_property(TARGET testCommandLineInterface APPEND PROPERTY
        COMPILE_DEFINITIONS
            OSIM_CLI_PATH="${osim_cli_path}"
            OSIM_ACTUATORS_LIB_PATH="${osim_actuators_lib_path}")
else()
    # We prefer to use generator expressions, but these are only available in
    # more recent versions of CMake.
    set_property(TARGET testCommandLineInterface APPEND PROPERTY
        COMPILE_DEFINITIONS
            OSIM_CLI_PATH="$<TARGET_FILE:opensim-cli>"
            OSIM_ACTUATORS_LIB_PATH="$<TARGET_FILE:osimActuators>")
endif()
