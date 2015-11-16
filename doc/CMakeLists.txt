install(FILES "BuildOpenSimPluginSteps.pdf"
        DESTINATION "${CMAKE_INSTALL_DOCDIR}")

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

    # We will populate this with the paths on which the doxygen target depends.
    set(DOXYGEN_TARGET_DEPENDENCIES)

    set(MATHJAX_ZIP_PATH "${PROJECT_BINARY_DIR}/MathJax.zip")

    ## LaTeX or MathJax
    # -----------------
    # Doxygen must render equations, and this can be done either with LaTeX or
    # MathJax. LaTeX is unpleasant to install on Windows, so we prefer MathJax.

    # There are two ways of using MathJax: from the internet, or locally.
    # We prefer using MathJax locally, but we have to download it first.
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
                "Overridden to OFF since we could not download MathJax."
                FORCE)
            list(GET MATHJAX_DOWNLOAD_STATUS 1 MATHJAX_DOWNLOAD_MESSAGE)
            message(STATUS
                "Could not download MathJax: ${MATHJAX_DOWNLOAD_MESSAGE}")
        endif()
    endif()

    if(${OPENSIM_DOXYGEN_USE_MATHJAX})
        # This means the user wants to use MathJax *and* we were able to
        # download it successfully.

        # See Doxyfile.in for a description of these variables.
        set(OPENSIM_DOXYGEN_USE_MATHJAX_DOXYFILE "YES")
        set(OPENSIM_DOXYGEN_MATHJAX_RELPATH "MathJax-2.5-latest")

    else()

        # Try using LATEX.
        find_package(LATEX)
        if(${LATEX_FOUND})
            set(OPENSIM_DOXYGEN_USE_MATHJAX_DOXYFILE "NO")
            message(STATUS "Using LATEX for Doxygen equations.")
        else()
            message(STATUS
                "Using MathJax Content Distribution Network for Doxygen.")
            set(OPENSIM_DOXYGEN_USE_MATHJAX_DOXYFILE "YES")
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

    # Configure doxygen configuration that's shared between both user
    # and developer configurations.
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doxyfile_shared.in
        ${CMAKE_CURRENT_BINARY_DIR}/doxyfile_shared @ONLY )
    
    ##############################
    # USER AND DEVELOPER DOXYGEN #
    ##############################
    set(DOXYGEN_AUDIENCES developer user)
    foreach(audience ${DOXYGEN_AUDIENCES})
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doxyfile_${audience}.in
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
    
        set(html_binary_dir "${PROJECT_BINARY_DIR}/html_${audience}/")
        file(MAKE_DIRECTORY ${html_binary_dir})

        # MathJax
        # -------
        if(${OPENSIM_DOXYGEN_USE_MATHJAX})
            # This means the user wants to use MathJax *and* we were able to
            # download it successfully.

            set(mathjax_unzipped_dir "${html_binary_dir}/MathJax-2.5-latest")

            # This creates a folder html_${audience}/MathJax-2.5-latest in the
            # build tree.
            add_custom_command(
                OUTPUT "${mathjax_unzipped_dir}"
                COMMENT "Unzipping MathJax.zip for ${audience} documentation"
                COMMAND ${CMAKE_COMMAND} -E tar -x "${MATHJAX_ZIP_PATH}" 
                WORKING_DIRECTORY "${html_binary_dir}"
                )

            # Add the MathJax unzipped directory to the dependencies for doxygen.
            list(APPEND DOXYGEN_TARGET_DEPENDENCIES "${mathjax_unzipped_dir}")
        endif()
    
        # Copy images directory.
        # ----------------------
        set(html_images_binary_dir "${html_binary_dir}/images")
        # There is an "images" directory containing the pictures needed by
        # the main page. Copy all the files into html_${audience}/images.
        add_custom_command(
            OUTPUT "${html_images_binary_dir}"
            COMMENT "Copying images for ${audience} Doxygen"
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                    "${CMAKE_CURRENT_SOURCE_DIR}/images"
                    "${html_images_binary_dir}"
            )
        list(APPEND DOXYGEN_TARGET_DEPENDENCIES "${html_images_binary_dir}")

    endforeach()

    # Define the doxygen target (now that we know the dependencies).
    # --------------------------------------------------------------
    # Doxyfile.in should take care of directing the output of Doxygen
    # to ${PROJECT_BINARY_DIR}/html_(user|developer).
    # Note that you have to invoke this "doxygen" target directly; it isn't run
    # by default.
    add_custom_target(doxygen
        SOURCES ${CMAKE_CURRENT_BINARY_DIR}/doxyfile_user
                ${CMAKE_CURRENT_BINARY_DIR}/doxyfile_developer
                ${CMAKE_CURRENT_BINARY_DIR}/doxyfile_shared
        COMMENT "Doxygen documentation"
        DEPENDS ${DOXYGEN_TARGET_DEPENDENCIES}
                )
    
    # Now create the command for running doxygen, which requires that we
    # have defined the doxygen target.
    foreach(audience ${DOXYGEN_AUDIENCES})

        # Run doxygen
        # -----------
        add_custom_command(TARGET doxygen
            COMMENT "Running Doxygen to build ${audience} documentation"
            COMMAND ${DOXYGEN_EXECUTABLE}
                    ${CMAKE_CURRENT_BINARY_DIR}/doxyfile_${audience})
    
        # Installation
        # ------------
        install(DIRECTORY   "${PROJECT_BINARY_DIR}/html_${audience}/"
                DESTINATION "${CMAKE_INSTALL_DOCDIR}/html_${audience}"
                )

    endforeach()

    # This is just a shortcut to the Doxygen index.html, for the user
    # documentation.
    install(FILES "OpenSimAPI.html" DESTINATION "${CMAKE_INSTALL_DOCDIR}")

endif()
