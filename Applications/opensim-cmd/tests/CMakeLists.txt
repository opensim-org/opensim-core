find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

OpenSimAddTest(NAME testCommandLineInterface)

add_dependencies(testCommandLineInterface opensim-cmd)

# Set preprocessor definitions used by the test to access other build products.
# OSIM_ACTUATORS_LIB_PATH is used to test the loading of plugins.
set_property(TARGET testCommandLineInterface APPEND PROPERTY
    COMPILE_DEFINITIONS OSIM_CLI_PATH="$<TARGET_FILE:opensim-cmd>"
                        OSIM_ACTUATORS_LIB_PATH="$<TARGET_FILE:osimActuators>")
