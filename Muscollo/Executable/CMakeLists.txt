
add_executable(opensim-muscollo opensim-muscollo.cpp)
target_link_libraries(opensim-muscollo osimMuscollo)

install(TARGETS opensim-muscollo DESTINATION bin)
set_target_properties(opensim-muscollo PROPERTIES
        DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
        PROJECT_LABEL "Executable - opensim-muscollo"
        FOLDER "Muscollo")


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
    set_property(TARGET opensim-muscollo APPEND PROPERTY
            INSTALL_RPATH "${rpath_macro}/${bin_dir_to_lib_dir}")
endif()
