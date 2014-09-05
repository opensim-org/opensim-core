INSTALL_FILES(/sdk/doc FILES "BuildOpenSimPluginSteps.pdf")

INCLUDE(FindDoxygen)

IF(DOXYGEN_EXECUTABLE-NOTFOUND)
ELSE(DOXYGEN_EXECUTABLE-NOTFOUND)
    SET(DOXY_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")

    # Configure linking to Simbody documentation.
    # Some of these variables are used in Doxyfile.in.
    # This variable must be a STRING; a PATH variable resolves //'s as a single
    # /, but we need to use double slashes for URL's.
    SET(OPENSIM_SIMBODY_DOXYGEN_LOCATION "${Simbody_DOXYGEN_DIR}"
        CACHE STRING
        "The location of Simbody's doxygen documentation. By default, OpenSim's
        Doxygen will link to the user's *local* Simbody documentation. However,
        when the OpenSim documentation is put online, there is no local Simbody
        Doxygen documentation. In this case, this variable can be set to a URL
        where one can find Simbody Doxygen documentation online. Delete from
        CMakeCache.txt to reset to default.")
    MARK_AS_ADVANCED(OPENSIM_SIMBODY_DOXYGEN_LOCATION)

    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in 
          ${DOXY_CONFIG}
          @ONLY )

    # The goal here is to run Doxygen to generate most of the API documentation
    # under "html" in the binary directory, then apply a few hand-tweaked 
    # hacks to it there.
    # The result should be fully-functional documentation in the binary 
    # directory (start with index.html) that can be examined while debugging 
    # Doxygen comments. Then when we do an INSTALL later (see below), we just 
    # need to copy over the binary html directory into sdk/doc.
    # (sherm 20120127)

    FILE(MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/html/")

    ###############
    # RUN DOXYGEN #
    ###############

    # Doxfile.in should take care of directing the output of Doxygen
    # to ${PROJECT_BINARY_DIR}/html. Note that you have to invoke this
    # "doxygen" target directly; it isn't run by default.
    ADD_CUSTOM_TARGET(doxygen ${DOXYGEN_EXECUTABLE} ${DOXY_CONFIG}) 

    # There is an "images" directory containing the pictures needed by
    # the main page Copy all the files into html/images.
    ADD_CUSTOM_COMMAND(TARGET doxygen POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/images"
        "${PROJECT_BINARY_DIR}/html/images" 
        )

    ################
    # INSTALLATION #
    ################
    INSTALL(DIRECTORY "${PROJECT_BINARY_DIR}/html/"
            DESTINATION "sdk/doc/html"
            )
    # This is just a shortcut to the Doxygen index.html.
    INSTALL(FILES "OpenSimAPI.html" DESTINATION "sdk/doc")

ENDIF(DOXYGEN_EXECUTABLE-NOTFOUND)
