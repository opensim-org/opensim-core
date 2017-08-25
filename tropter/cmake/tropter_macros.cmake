# These CMake functions serve to reduce duplication across CMakeLists.txt files.

include(CMakeParseArguments)

# Create an exectuable for the file ${TEST_NAME}.cpp, which depends on
# libraries ${LIB_DEPENDENCIES}. Also create a CTest test for this executable.
# We expect that the test cases use the Catch testing framework, and
# therefore we pass some command-line flags that Catch interprets.
function(tropter_add_test TEST_NAME LIB_DEPENDENCIES)
    add_executable(${TEST_NAME} ${TEST_NAME}.cpp)
    # To organize targets in Visual Studio's Solution Explorer.
    set_target_properties(${TEST_NAME} PROPERTIES FOLDER "tropter/Tests")
    target_link_libraries(${TEST_NAME} ${LIB_DEPENDENCIES})
    # TODO Tropter shouldn't know that it's in a larger project.
    target_include_directories(${TEST_NAME}
            PRIVATE "${CMAKE_SOURCE_DIR}/tropter/external/catch")
    if(MUSCOLLO_WITH_SNOPT)
        target_compile_definitions(${TEST_NAME} PRIVATE MUSCOLLO_WITH_SNOPT)
    endif()
    add_test(NAME ${TEST_NAME}
             COMMAND ${TEST_NAME} --use-colour yes --durations yes)
    #if(WIN32) # Instead, we are copying dependencies' DLLs into Tropter.
    #    set_property(TEST ${TEST_NAME} APPEND PROPERTY
    #        ENVIRONMENT "PATH=${IPOPT_DIR}/bin\;${ADOLC_DIR}/bin")
    #endif()
endfunction()

# Copy DLL files from a dependency's installation into Tropter's
# build and install directories. This is a Windows-specific function enabled 
# only on Windows. The intention is to allow the runtime loader to find all 
# the required DLLs without editing the PATH environment variable.
# Copied from OpenSimMacros.cmake.
# In the future, we could use the VS_USER_PROPS_CXX property instead
# https://gitlab.kitware.com/cmake/cmake/commit/ef121ca0c33fb4931007c38b22c046998694b052
function(tropter_copy_dlls DEP_NAME DEP_INSTALL_DIR)
    # On Windows, copy dlls into the Tropter binary directory.
    if(WIN32)
        file(GLOB_RECURSE DLLS ${DEP_INSTALL_DIR}/*.dll)
        if(NOT DLLS)
            message(FATAL_ERROR "Zero DLLs found in directory "
                                "${DEP_INSTALL_DIR}.")
        endif()
        set(DEST_DIR "${CMAKE_BINARY_DIR}/${CMAKE_CFG_INTDIR}")
        # https://stackoverflow.com/questions/21198030/installfiles-cmake-cfg-intdir-abc-win-dll-destination-bin
        set(DEST_DIR_FOR_INST "${CMAKE_BINARY_DIR}/$<CONFIG>")
        set(DLL_NAMES "")
        foreach(DLL IN LISTS DLLS)
            get_filename_component(DLL_NAME ${DLL} NAME)
            list(APPEND DLLS_DEST "${DEST_DIR}/${DLL_NAME}")
            list(APPEND DLLS_FOR_INST "${DEST_DIR_FOR_INST}/${DLL_NAME}")
            set(DLL_NAMES "${DLL_NAMES} ${DLL_NAME}")
        endforeach()
        add_custom_command(OUTPUT ${DLLS_DEST}
            COMMAND ${CMAKE_COMMAND} -E make_directory ${DEST_DIR}
            COMMAND ${CMAKE_COMMAND} -E copy ${DLLS} ${DEST_DIR}
            COMMENT "Copying DLLs from ${DEP_INSTALL_DIR} to ${DEST_DIR}:${DLL_NAMES}")
        add_custom_target(Copy_${DEP_NAME}_DLLs ALL DEPENDS ${DLLS_DEST})
        set_target_properties(Copy_${DEP_NAME}_DLLs PROPERTIES
            PROJECT_LABEL "Copy ${DEP_NAME} DLLs" FOLDER "tropter")
        if(TROPTER_COPY_DEPENDENCIES)
            install(FILES ${DLLS_FOR_INST} DESTINATION ${CMAKE_INSTALL_BINDIR})
        endif()
    endif()
endfunction()
