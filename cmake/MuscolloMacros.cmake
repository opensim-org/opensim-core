# These CMake functions serve to reduce duplication across CMakeLists.txt files.

include(CMakeParseArguments)

# Copy DLL files from a dependency's installation into Tropter's
# build and install directories. This is a Windows-specific function enabled 
# only on Windows. The intention is to allow the runtime loader to find all 
# the required DLLs without editing the PATH environment variable.
# Copied from OpenSimMacros.cmake.
function(MuscolloCopyDLLs DEP_NAME DEP_INSTALL_DIR)
    # On Windows, copy dlls into the Tropter binary directory.
    if(WIN32)
        file(GLOB_RECURSE DLLS ${DEP_INSTALL_DIR}/*.dll)
        if(NOT DLLS)
            message(FATAL_ERROR "Zero DLLs found in directory "
                                "${DEP_INSTALL_DIR}.")
        endif()
        set(DEST_DIR "${CMAKE_BINARY_DIR}")
        if(MSVC)
        	set(DEST_DIR "${DEST_DIR}/${CMAKE_CFG_INTDIR}")
        endif()
        add_custom_command(OUTPUT ${DLLS}
                           COMMAND ${CMAKE_COMMAND} -E make_directory ${DEST_DIR}
                           COMMAND ${CMAKE_COMMAND} -E copy ${DLLS} ${DEST_DIR}
                           COMMENT "Copying ${DLLS} to ${DEST_DIR}.")
        add_custom_target(Copy_${DEP_NAME}_DLLs ALL DEPENDS ${DLLS})
        set_target_properties(Copy_${DEP_NAME}_DLLs PROPERTIES
            PROJECT_LABEL "Copy ${DEP_NAME} DLLs" FOLDER "Muscollo")
        if(MUSCOLLO_COPY_DEPENDENCIES)
            install(FILES ${DLLS} DESTINATION ${CMAKE_INSTALL_BINDIR})
        endif()
    endif()
endfunction()
