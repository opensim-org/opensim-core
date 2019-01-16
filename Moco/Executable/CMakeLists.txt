
add_executable(opensim-moco opensim-moco.cpp)
target_link_libraries(opensim-moco osimMoco)

install(TARGETS opensim-moco DESTINATION bin)
set_target_properties(opensim-moco PROPERTIES
        DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
        PROJECT_LABEL "Executable - opensim-moco"
        FOLDER "Moco")


# RPATH
# -----
if(UNIX)
    if(APPLE)
        set(rpath_macro "\@executable_path")
    elseif(UNIX)
        set(rpath_macro "\$ORIGIN")
    endif()
    # bin_dir_to_install_dir is most likely "../"
    file(RELATIVE_PATH bin_dir_to_install_dir
            "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
            "${CMAKE_INSTALL_PREFIX}")
    set(bin_dir_to_lib_dir
            "${bin_dir_to_install_dir}${CMAKE_INSTALL_LIBDIR}")
    set_property(TARGET opensim-moco APPEND PROPERTY
            INSTALL_RPATH "${rpath_macro}/${bin_dir_to_lib_dir}")
endif()
