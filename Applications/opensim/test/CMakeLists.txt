
OpenSimAddTests(
    TESTPROGRAMS testCommandLineInterface
    # Don't need OpenSim since we only interact with OpenSim through the
    # command line tool. But we use Simbody for its Testing.h.
    LINKLIBS SimTKsimbody
    )

add_dependencies(testCommandLineInterface opensim-cli)

# Set macros used by the test to access other build products.
if(${CMAKE_VERSION} VERSION_LESS 3.0)
    get_target_property(osim_cli_path opensim-cli LOCATION_${CMAKE_BUILD_TYPE})
    get_target_property(osim_actuators_lib_path osimActuators
        LOCATION_${CMAKE_BUILD_TYPE})
else()
    # We prefer to use generator expressions, but these are only available in
    # more recent versions of CMake.
    set(osim_cli_path "$<TARGET_FILE:opensim-cli>")
    set(osim_actuators_lib_path "$<TARGET_FILE:osimActuators>")
endif()

set_property(TARGET testCommandLineInterface APPEND PROPERTY
    COMPILE_DEFINITIONS OSIM_CLI_PATH="${osim_cli_path}"
                        OSIM_ACTUATORS_LIB_PATH="${osim_actuators_lib_path}")
