
OpenSimAddTests(
    TESTPROGRAMS testCommandLineInterface
    # Don't need OpenSim since we only interact with OpenSim through the
    # command line tool. But we use Simbody for its Testing.h.
    LINKLIBS SimTKcommon
    )

add_dependencies(testCommandLineInterface opensim-cli)

# Set preprocessor definitions used by the test to access other build products.
set(osim_cli_path "")
set(osim_actuators_lib_path "")

set_property(TARGET testCommandLineInterface APPEND PROPERTY
    COMPILE_DEFINITIONS OSIM_CLI_PATH="$<TARGET_FILE:opensim-cli>"
                        OSIM_ACTUATORS_LIB_PATH="$<TARGET_FILE:osimActuators>")
