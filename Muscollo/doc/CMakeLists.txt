find_package(Doxygen)

# Eventually, Muscollo's doxygen will simply become part of OpenSim's.
if(DOXYGEN_FOUND)

    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/doxyfile.in"
            "${CMAKE_CURRENT_BINARY_DIR}/doxyfile" @ONLY)

    add_custom_target(Muscollo_doxygen
            COMMENT "Muscollo doxygen documentation"
            SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/doxyfile.in")
    set_target_properties(Muscollo_doxygen PROPERTIES
            PROJECT_LABEL "Muscollo doxygen" FOLDER "Muscollo")
    add_custom_command(TARGET Muscollo_doxygen
            COMMENT "Running doxygen to build Muscollo documentation."
            COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doxyfile)

    install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html/"
            DESTINATION "${CMAKE_INSTALL_DOCDIR}/MuscolloAPI")

endif()
