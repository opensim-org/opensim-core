file(TO_CMAKE_PATH "$ENV{OPENSIM_GEOMETRY_DIR}" OPENSIM_GEOMETRY_DIR)
add_compile_definitions(OPENSIM_GEOMETRY_DIR="${OPENSIM_GEOMETRY_DIR}")
message(STATUS ${OPENSIM_GEOMETRY_DIR})

#Add all subdirectories
file(GLOB EXAMPLE_SUBDIRS "*")
foreach(subdir ${EXAMPLE_SUBDIRS})
    if(IS_DIRECTORY ${subdir})
        message("Adding ${subdir}")
        add_subdirectory(${subdir})
    endif()
endforeach()