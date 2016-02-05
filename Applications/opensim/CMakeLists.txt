
# TODO if python exists.
add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/opensim"
    COMMENT "Building the opensim command line tool with PyInstaller."
    COMMAND pyinstaller --clean --noconfirm
                --debug --log-level=DEBUG
                # Package the executable as a single file:
                --onefile
                # Location of output:
                --distpath "${CMAKE_CURRENT_BINARY_DIR}"
                # Add the location of the opensim package to the PYTHONPATH.
                --paths "${CMAKE_BINARY_DIR}/Bindings/Python/$<CONFIG>"
                # The python script to convert into a native executable:
                "${CMAKE_CURRENT_SOURCE_DIR}/opensim.py"
                )

add_custom_target(CommandLineTool
    DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/opensim"
    SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/opensim.py"
    )

set_target_properties(CommandLineTool PROPERTIES
    PROJECT_LABEL "Command Line Tool"
    )

add_test(NAME CommandLineTool
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    COMMAND "./opensim"
    )


# We can only build the command line tool if we've built the python bindings.
add_dependencies(CommandLineTool PythonBindings)
