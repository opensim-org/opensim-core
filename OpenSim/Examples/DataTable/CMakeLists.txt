set(EXAMPLES_TO_COMPILE example1.cpp
                        example2.cpp
)

foreach(EXAMPLE_PROG ${EXAMPLES_TO_COMPILE})
    get_filename_component(EXAMPLE_ROOT ${EXAMPLE_PROG} NAME_WE)
    add_executable(${EXAMPLE_ROOT}DataTable ${EXAMPLE_PROG})
    target_link_libraries(${EXAMPLE_ROOT}DataTable ${Simbody_LIBRARIES} osimCommon)
    set_target_properties(${EXAMPLE_ROOT}DataTable PROPERTIES FOLDER "Examples")
endforeach(EXAMPLE_PROG ${EXAMPLES_TO_COMPILE})
