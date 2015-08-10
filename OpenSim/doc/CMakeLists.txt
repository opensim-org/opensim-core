install_files(/sdk/doc FILES "BuildOpenSimPluginSteps.pdf")

find_package(Doxygen)

if(${DOXYGEN_FOUND})
    set(DOXY_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")

    # Set some useful variables.
    # --------------------------
    # The goal here is to run Doxygen to generate most of the API documentation
    # under "html" in the binary directory, then apply a few hand-tweaked 
    # hacks to it there.
    # The result should be fully-functional documentation in the binary 
    # directory (start with index.html) that can be examined while debugging 
    # Doxygen comments. Then when we do an INSTALL later (see below), we just 
    # need to copy over the binary html directory into sdk/doc.
    # (sherm 20120127)
    set(HTML_BINARY_DIR "${PROJECT_BINARY_DIR}/html/")
    file(MAKE_DIRECTORY "${HTML_BINARY_DIR}")

    set(HTML_IMAGES_BINARY_DIR "${HTML_BINARY_DIR}/images")
    set(DOXYGEN_TARGET_DEPENDENCIES "${HTML_IMAGES_BINARY_DIR}")

    set(MATHJAX_ZIP_PATH "${PROJECT_BINARY_DIR}/MathJax.zip")

    ## LaTeX or MathJax
    # -----------------
    # Doxygen must render equations, and this can be done either with LaTeX or
    # MathJax. LaTeX is unpleasant to install on Windows, so we prefer MathJax.

    # There are two ways of using MathJax: from the internet, or locally.
    # We prefer using MathJax locally, bt we have to download it first.
    # We download a 50 mb MathJax zip and unzip it (150 mb).
    # The zip can be thrown away, but the 150 mb MathJax directory must stay
    # with the Doxygen documentation.

    # If we are unable to download MathJax, and the user has LaTeX, we fall
    # back onto LaTeX. If the user has neither MathJax *or* LaTeX, we fall back
    # on using MathJax from the internet instead of downloading it.

    # This block of commands is all executing during the configuring phase.
    if(${OPENSIM_DOXYGEN_USE_MATHJAX} AND NOT EXISTS "${MATHJAX_ZIP_PATH}")
        message(STATUS "Trying to download MathJax for Doxygen...")
        file(DOWNLOAD https://github.com/mathjax/MathJax/archive/v2.5-latest.zip
            "${MATHJAX_ZIP_PATH}"
            #SHOW_PROGRESS Clutters the CMake output.
            STATUS MATHJAX_DOWNLOAD_STATUS
            )

        # Did we download MathJax successfully?
        # Get first element of the status.
        list(GET MATHJAX_DOWNLOAD_STATUS 0 MATHJAX_DOWNLOAD_RETURN)
        if(${MATHJAX_DOWNLOAD_RETURN} EQUAL 0)
            message(STATUS "Successfully downloaded MathJax.")
        else()
            # Must modify the cache variable so we don't end up here after
            # running the CMake configuring step multiple times.
            set(OPENSIM_DOXYGEN_USE_MATHJAX OFF CACHE BOOL
                "Overriden to OFF since we could not download MathJax."
                FORCE)
            list(GET MATHJAX_DOWNLOAD_STATUS 1 MATHJAX_DOWNLOAD_MESSAGE)
            message(STATUS
                "Could not download MathJax: ${MATHJAX_DOWNLOAD_MESSAGE}")
        endif()
    endif()

    if(${OPENSIM_DOXYGEN_USE_MATHJAX})
        # This means the user wants to use MathJax *and* we were able to
        # download it successfully.

        set(MATHJAX_UNZIPPED_DIR "${HTML_BINARY_DIR}/MathJax-2.5-latest")

        # This creates a folder html/MathJax-2.5-latest in the build tree.
        add_custom_command(
            OUTPUT "${MATHJAX_UNZIPPED_DIR}"
            COMMENT "Unzipping MathJax.zip."
            COMMAND ${CMAKE_COMMAND} -E tar -x "${MATHJAX_ZIP_PATH}" 
            WORKING_DIRECTORY "${HTML_BINARY_DIR}"
            )

        # Add the MathJax unzipped directory to the dependencies for doxygen.
        list(APPEND DOXYGEN_TARGET_DEPENDENCIES "${MATHJAX_UNZIPPED_DIR}")

        # See Doxyfile.in for a description of these variables.
        set(OPENSIM_DOXYGEN_USE_MATHJAX "YES")
        set(OPENSIM_DOXYGEN_MATHJAX_RELPATH "MathJax-2.5-latest")

    else()

        # Try using LATEX.
        find_package(LATEX)
        if(${LATEX_FOUND})
            set(OPENSIM_DOXYGEN_USE_MATHJAX "NO")
            message(STATUS "Using LATEX for Doxygen equations.")
        else()
            message(STATUS
                "Cound not find LATEX. Using MathJax Content Distribution Network.")
            set(OPENSIM_DOXYGEN_USE_MATHJAX "YES")
            set(OPENSIM_DOXYGEN_MATHJAX_RELPATH
                "https://cdn.mathjax.org/mathjax/latest")
        endif()

    endif()

    # Configure linking to Simbody documentation.
    # -------------------------------------------
    # This variable must be a STRING; a PATH variable resolves //'s as a single
    # /, but we need to use double slashes for URL's.
    set(OPENSIM_SIMBODY_DOXYGEN_LOCATION "${Simbody_DOXYGEN_DIR}"
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

    mark_as_advanced(OPENSIM_SIMBODY_DOXYGEN_LOCATION)

    # This variable is used in Doxyfile.in.
    set(OPENSIM_SIMBODY_TAGFILE_CONFIG "")
    if(Simbody_DOXYGEN_TAGFILE)
        # The variable above is only defined if Simbody's Doxygen documentation
        # was (built and) installed.
        set(OPENSIM_SIMBODY_TAGFILE_CONFIG
            "${Simbody_DOXYGEN_TAGFILE}=${OPENSIM_SIMBODY_DOXYGEN_LOCATION}")
    endif()

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in 
          ${DOXY_CONFIG}
          @ONLY )

    ###############
    # RUN DOXYGEN #
    ###############

    # Doxfile.in should take care of directing the output of Doxygen
    # to ${PROJECT_BINARY_DIR}/html. Note that you have to invoke this
    # "doxygen" target directly; it isn't run by default.
    add_custom_target(doxygen ${DOXYGEN_EXECUTABLE} ${DOXY_CONFIG}
        COMMENT "Generating Doxygen documentation",
        DEPENDS ${DOXYGEN_TARGET_DEPENDENCIES}
        )

    # There is an "images" directory containing the pictures needed by
    # the main page Copy all the files into html/images.
    add_custom_command(
        OUTPUT "${HTML_IMAGES_BINARY_DIR}"
        COMMENT "Copying images for Doxygen."
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/images"
        "${HTML_IMAGES_BINARY_DIR}"
        )

    ################
    # INSTALLATION #
    ################
    install(DIRECTORY "${PROJECT_BINARY_DIR}/html/"
            DESTINATION "sdk/doc/html"
            )
    # This is just a shortcut to the Doxygen index.html.
    install(FILES "OpenSimAPI.html" DESTINATION "sdk/doc")

endif()
