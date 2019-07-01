find_package(Doxygen)

# Eventually, Moco's doxygen will simply become part of OpenSim's.
if(DOXYGEN_FOUND)

    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/doxyfile.in"
            "${CMAKE_CURRENT_BINARY_DIR}/doxyfile" @ONLY)

    add_custom_target(Moco_doxygen
            COMMENT "Moco doxygen documentation"
            SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/doxyfile.in")
    set_target_properties(Moco_doxygen PROPERTIES
            PROJECT_LABEL "Moco doxygen" FOLDER "Moco")
    add_custom_command(TARGET Moco_doxygen
            COMMENT "Running doxygen to build Moco documentation."
            COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doxyfile)

    set(html_binary_dir "${CMAKE_CURRENT_BINARY_DIR}/html/")
    # So that we can build the install target even if doxygen was not built.
    file(MAKE_DIRECTORY ${html_binary_dir})

    install(DIRECTORY "${html_binary_dir}"
            DESTINATION "${CMAKE_INSTALL_DOCDIR}/MocoAPI")
    install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/MocoAPI.html"
            DESTINATION "${CMAKE_INSTALL_DOCDIR}/")

endif()
