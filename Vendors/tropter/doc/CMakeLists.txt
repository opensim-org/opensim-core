find_package(Doxygen)
set_package_properties(Doxygen PROPERTIES
        URL http://www.doxygen.org
        TYPE OPTIONAL
        PURPOSE "Generate C++ API documentation")

if(DOXYGEN_FOUND)

    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/doxyfile.in"
            "${CMAKE_CURRENT_BINARY_DIR}/doxyfile" @ONLY)

    add_custom_target(tropter_doxygen
            COMMENT "tropter doxygen documentation"
            SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/doxyfile.in")
    set_target_properties(tropter_doxygen PROPERTIES
            PROJECT_LABEL "tropter doxygen" FOLDER "tropter")
    add_custom_command(TARGET tropter_doxygen
            COMMENT "Running doxygen to build tropter documentation."
            COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doxyfile)

    set(html_binary_dir "${CMAKE_CURRENT_BINARY_DIR}/html/")
    # So that we can build the install target even if doxygen was not built.
    file(MAKE_DIRECTORY ${html_binary_dir})

    # TODO When tropter is moved to a separate repository, remove "tropter_".
    # install(DIRECTORY "${html_binary_dir}"
    #         DESTINATION "${CMAKE_INSTALL_DOCDIR}/tropter_API")

    # install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/tropter_API.html"
    #         DESTINATION "${CMAKE_INSTALL_DOCDIR}/")

endif()
