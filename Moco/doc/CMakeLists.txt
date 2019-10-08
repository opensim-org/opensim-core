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
            DESTINATION "${CMAKE_INSTALL_DOCDIR}/MocoDocumentation")
    install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/MocoDocumentation.html"
            DESTINATION "${CMAKE_INSTALL_DOCDIR}/")

endif()

find_package(LATEX COMPONENTS PDFLATEX)

set_package_properties(LATEX PROPERTIES
        URL https://www.latex-project.org/
        TYPE OPTIONAL
        PURPOSE "Building the MocoCheatSheet PDF.")

if(LATEX_PDFLATEX_FOUND)
    add_custom_target(MocoCheatSheet
            COMMENT "MocoCheatSheet LaTeX PDF"
            SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/MocoCheatSheet.tex")
    set_target_properties(MocoCheatSheet PROPERTIES
            PROJECT_LABEL "MocoCheatSheet" FOLDER "Moco")
    add_custom_command(TARGET MocoCheatSheet
            COMMENT "Running pdflatex to build MocoCheatSheet."
            COMMAND "${PDFLATEX_COMPILER}"
                    "-output-directory='${CMAKE_CURRENT_BINARY_DIR}'"
                    "MocoCheatSheet.tex"
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/MocoCheatSheet.pdf"
            DESTINATION "${CMAKE_INSTALL_DOCDIR}/")
endif()
