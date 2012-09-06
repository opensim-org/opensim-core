INCLUDE(FindDoxygen)

IF(DOXYGEN_EXECUTABLE-NOTFOUND)
ELSE(DOXYGEN_EXECUTABLE-NOTFOUND)
    SET(DOXY_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")

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
    FILE(MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/html/images/")

    ###############
    # RUN DOXYGEN #
    ###############
    # Doxfile.in should take care of directing the output of Doxygen
    # to ${PROJECT_BINARY_DIR}/html. Note that you have to invoke this
    # "doxygen" target directly; it isn't run by default.
    ADD_CUSTOM_TARGET(doxygen ${DOXYGEN_EXECUTABLE} ${DOXY_CONFIG}) 

    # There is an "images" directory containing the pictures needed by
    # the main page Copy all the files into html/images.
    FILE(GLOB ALL_IMAGES 
         "${CMAKE_CURRENT_SOURCE_DIR}/OpenSim/doc/images/*.gif"
         "${CMAKE_CURRENT_SOURCE_DIR}/OpenSim/doc/images/*.png")

    FOREACH(image_pathname ${ALL_IMAGES})
        # Retrieve just the filename (with extension).
        GET_FILENAME_COMPONENT(image_name ${image_pathname} NAME)
        FILE(TO_NATIVE_PATH "${image_pathname}" 
            image_file_src)
        FILE(TO_NATIVE_PATH "${PROJECT_BINARY_DIR}/html/images/${image_name}"
            image_file_dest)
        ADD_CUSTOM_COMMAND(TARGET doxygen POST_BUILD
           COMMAND ${NATIVE_COPY_CMD} "${image_file_src}" "${image_file_dest}"
           COMMENT "Copy ${image_file_src} -> ${image_file_dest}."
           VERBATIM)
    ENDFOREACH()

    ################
    # INSTALLATION #
    ################
    INSTALL(DIRECTORY "${PROJECT_BINARY_DIR}/html/"
            DESTINATION "sdk/doc/html"
            )
    # This is just a shortcut to the Doxygen index.html.
    INSTALL(FILES "OpenSimAPI.html" DESTINATION "sdk/doc")

ENDIF(DOXYGEN_EXECUTABLE-NOTFOUND)
