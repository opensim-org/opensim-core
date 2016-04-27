
OpenSimAddTests(
    TESTPROGRAMS testCommandLineInterface
    # Don't need OpenSim since we only interact with OpenSim through the
    # command line tool. But we use Simbody for its Testing.h.
    LINKLIBS SimTKsimbody
    )

add_dependencies(testCommandLineInterface opensim-cli)

# TOOD add in quotes! will not work for filepaths that have spaces.
set_property(TARGET testCommandLineInterface APPEND PROPERTY
    COMPILE_DEFINITIONS OSIM_CLI_PATH=$<TARGET_FILE:opensim-cli>
                        OSIM_ACTUATORS_LIB_PATH=$<TARGET_FILE:osimActuators>)
