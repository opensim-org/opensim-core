set(EXAMPLES_TO_COMPILE StaticOptimizationExample.cpp
)

foreach(EXAMPLE_PROG ${EXAMPLES_TO_COMPILE})
    get_filename_component(EXAMPLE_ROOT ${EXAMPLE_PROG} NAME_WE)
    add_executable(${EXAMPLE_ROOT} ${EXAMPLE_PROG})
    target_link_libraries(${EXAMPLE_ROOT} ${Simbody_LIBRARIES} osimCommon osimTools)
    set_target_properties(${EXAMPLE_ROOT} PROPERTIES FOLDER "Examples")

    file(GLOB ADDITIONAL_FILES *.osim *.xml *.mot)

    foreach(ADDITIONAL_FILE ${ADDITIONAL_FILES})
        get_filename_component(ADDITIONAL_FILE_NAME ${ADDITIONAL_FILE} NAME)
        add_custom_command(
            TARGET ${EXAMPLE_ROOT}
            COMMAND ${CMAKE_COMMAND}
            ARGS -E copy
            ${ADDITIONAL_FILE}
            ${CMAKE_BINARY_DIR}/OpenSim/Examples/${EXAMPLE_ROOT}/${ADDITIONAL_FILE_NAME})
    endforeach(ADDITIONAL_FILE)

    # Doesn't work...
    add_custom_target(
        TARGET ${EXAMPLE_ROOT}
        COMMAND ${CMAKE_COMMAND} -E make_directory
        ${CMAKE_BINARY_DIR}/OpenSim/Examples/${EXAMPLE_ROOT}/Results)
endforeach(EXAMPLE_PROG ${EXAMPLES_TO_COMPILE})





