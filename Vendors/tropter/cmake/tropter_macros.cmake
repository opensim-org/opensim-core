# These CMake functions serve to reduce duplication across CMakeLists.txt files.

include(CMakeParseArguments)

# Create an exectuable for the file ${TEST_NAME}.cpp, which depends on
# libraries ${LIB_DEPENDENCIES}. Also create a CTest test for this executable.
# We expect that the test cases use the Catch testing framework, and
# therefore we pass some command-line flags that Catch interprets.
function(tropter_add_test)
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs LIB_DEPENDS)
    cmake_parse_arguments(TROPTEST
            "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    add_executable(${TROPTEST_NAME} ${TROPTEST_NAME}.cpp)
    # To organize targets in Visual Studio's Solution Explorer.
    set_target_properties(${TROPTEST_NAME} PROPERTIES FOLDER "tropter/Tests")
    target_link_libraries(${TROPTEST_NAME} tropter ${TROPTEST_LIB_DEPENDS})
    # TODO Tropter shouldn't know that it's in a larger project.
    target_include_directories(${TROPTEST_NAME}
        PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../external/catch")
    if(TROPTER_WITH_SNOPT)
        target_compile_definitions(${TROPTEST_NAME} PRIVATE TROPTER_WITH_SNOPT)
    endif()
    add_test(NAME ${TROPTEST_NAME}
             COMMAND ${TROPTEST_NAME} --use-colour yes --durations yes)
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
function(tropter_copy_dlls)
    set(options DONT_INSTALL_DLLS)
    set(oneValueArgs DEP_NAME DEP_INSTALL_DIR)
    set(multiValueArgs)
    cmake_parse_arguments(TROPDLL
            "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    # On Windows, copy dlls into the Tropter binary directory.
    if(WIN32)
        file(GLOB_RECURSE DLLS ${TROPDLL_DEP_INSTALL_DIR}/*.dll)
        if(NOT DLLS)
            message(FATAL_ERROR "Zero DLLs found in directory "
                "${TROPDLL_DEP_INSTALL_DIR}.")
        endif()
        set(DEST_DIR "${CMAKE_BINARY_DIR}/${CMAKE_CFG_INTDIR}")
        foreach(DLL IN LISTS DLLS)
            get_filename_component(DLL_NAME ${DLL} NAME)
            list(APPEND DLLS_DEST "${DEST_DIR}/${DLL_NAME}")
            add_custom_command(OUTPUT "${DEST_DIR}/${DLL_NAME}"
                COMMAND ${CMAKE_COMMAND} -E make_directory ${DEST_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy ${DLL} ${DEST_DIR}
                DEPENDS ${DLL}
                COMMENT "Copying ${DLL_NAME} from ${TROPDLL_DEP_INSTALL_DIR} to ${DEST_DIR}.")
        endforeach()
        add_custom_target(Copy_${TROPDLL_DEP_NAME}_DLLs ALL DEPENDS ${DLLS_DEST})
        set_target_properties(Copy_${TROPDLL_DEP_NAME}_DLLs PROPERTIES
            PROJECT_LABEL "Copy ${TROPDLL_DEP_NAME} DLLs" FOLDER "tropter")
        if(TROPTER_COPY_DEPENDENCIES AND NOT TROPDLL_DONT_INSTALL_DLLS)
            install(FILES ${DLLS} DESTINATION ${CMAKE_INSTALL_BINDIR})
        endif()
    endif()
endfunction()
