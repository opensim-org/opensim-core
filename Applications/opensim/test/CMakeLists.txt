
OpenSimAddTests(
    TESTPROGRAMS testCommandLineInterface
    # Don't need OpenSim since we only interact with OpenSim through the
    # command line tool. But we use Simbody for its Testing.h.
    LINKLIBS SimTKsimbody
    )

add_dependencies(testCommandLineInterface opensim-cli)

# TODO different with Xcode or MSVC.
set_target_properties(testCommandLineInterface PROPERTIES
    COMPILE_DEFINITIONS "OSIM_CLI_PATH=$<TARGET_FILE:opensim-cli>")
