INSTALL_FILES(/sdk/doc FILES "BuildOpenSimPluginSteps.pdf")

INCLUDE(FindDoxygen)

IF(DOXYGEN_EXECUTABLE-NOTFOUND)
ELSE(DOXYGEN_EXECUTABLE-NOTFOUND)

    ###############################################
    # Configure linking to Simbody documentation. #
    ###############################################

    # This variable must be a STRING; a PATH variable resolves //'s as a single
    # /, but we need to use double slashes for URL's.
    SET(OPENSIM_SIMBODY_DOXYGEN_LOCATION "${Simbody_DOXYGEN_DIR}"
        CACHE STRING "The location of Simbody's doxygen documentation.
        To link to Simbody Doxygen documentation, you must build
        *and* install Simbody Doxygen documentation. By default,
        OpenSim's Doxygen documentation will link to your *local* Simbody
        Doxygen documentation, and this variable is automatically populated 
        with the location of Simbody's Doyxgen files.
        However, when the OpenSim Doxygen documentation is put online, 
        there is no 'local' Simbody Doxygen documentation. In this case, 
        set this variable to a URL where one can find the correct 
        Simbody Doxygen documentation online. To reset to default
        (the local location of Simbody's Doxygen files), remove this
        variable from your CMakeCache.txt, and it will be 
        automatically populated again.")

    MARK_AS_ADVANCED(OPENSIM_SIMBODY_DOXYGEN_LOCATION)

    # This variable is used in Doxyfile.in.
    SET(OPENSIM_SIMBODY_TAGFILE_CONFIG "")
    IF(Simbody_DOXYGEN_TAGFILE)
        # The variable above is only defined if Simbody's Doxygen documentation
        # was (built and) installed.
        SET(OPENSIM_SIMBODY_TAGFILE_CONFIG
            "${Simbody_DOXYGEN_TAGFILE}=${OPENSIM_SIMBODY_DOXYGEN_LOCATION}")
    ENDIF()

    # Doxfile.in should take care of directing the output of Doxygen
    # to ${PROJECT_BINARY_DIR}/html. Note that you have to invoke this
    # "doxygen" target directly; it isn't run by default.
    ADD_CUSTOM_TARGET(doxygen
        SOURCES ${CMAKE_CURRENT_BINARY_DIR}/doxyfile_user
                ${CMAKE_CURRENT_BINARY_DIR}/doxyfile_developer
                ${CMAKE_CURRENT_BINARY_DIR}/doxyfile_shared
        COMMENT "Generate Doxygen documentation"
                )

    # Configure doxygen configuration that's shared between both user
    # and developer configurations.
    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxyfile_shared.in
        ${CMAKE_CURRENT_BINARY_DIR}/doxyfile_shared @ONLY )
    
    ##############################
    # USER AND DEVELOPER DOXYGEN #
    ##############################

    FOREACH(audience developer user)
        CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxyfile_${audience}.in
            ${CMAKE_CURRENT_BINARY_DIR}/doxyfile_${audience} @ONLY )

        # The goal here is to run Doxygen to generate most of the API
        # documentation under "html" in the binary directory, then apply a few
        # hand-tweaked hacks to it there.
        # The result should be fully-functional documentation in the binary
        # directory (start with index.html) that can be examined while
        # debugging Doxygen comments. Then when we do an INSTALL later (see
        # below), we just need to copy over the binary html directory into
        # sdk/doc.
        # (sherm 20120127)
        # ^ There are now two builds of doxygen, so the above comment now
        # applies to both sets.
        # (chrisdembia 20150507)
    
        FILE(MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/html_${audience}/")
    
        ###############
        # RUN DOXYGEN #
        ###############
    
        # There is an "images" directory containing the pictures needed by
        # the main page. Copy all the files into html_${audience}/images.
        ADD_CUSTOM_COMMAND(TARGET doxygen PRE_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/images"
            "${PROJECT_BINARY_DIR}/html_${audience}/images" 
            )
    
        # Run doxygen.
        ADD_CUSTOM_COMMAND(TARGET doxygen COMMAND ${DOXYGEN_EXECUTABLE}
            ${CMAKE_CURRENT_BINARY_DIR}/doxyfile_${audience})
    
        ################
        # INSTALLATION #
        ################
        INSTALL(DIRECTORY "${PROJECT_BINARY_DIR}/html_${audience}/"
                DESTINATION "sdk/doc/html_${audience}"
                )

    ENDFOREACH()

    # This is just a shortcut to the Doxygen index.html, for the user
    # documentation.
    INSTALL(FILES "OpenSimAPI.html" DESTINATION "sdk/doc")

ENDIF(DOXYGEN_EXECUTABLE-NOTFOUND)
