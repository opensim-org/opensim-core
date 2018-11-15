
OpenSimAddTests(
    TESTPROGRAMS testCommandLineInterface
    # Don't need OpenSim since we only interact with OpenSim through the
    # command line tool. But we use Simbody for its Testing.h.
    LINKLIBS SimTKcommon
    )

add_dependencies(testCommandLineInterface opensim-cmd)

# Set preprocessor definitions used by the test to access other build products.
# OSIM_ACTUATORS_LIB_PATH is used to test the loading of plugins.
set_property(TARGET testCommandLineInterface APPEND PROPERTY
    COMPILE_DEFINITIONS OSIM_CLI_PATH="$<TARGET_FILE:opensim-cmd>"
                        OSIM_ACTUATORS_LIB_PATH="$<TARGET_FILE:osimActuators>")
