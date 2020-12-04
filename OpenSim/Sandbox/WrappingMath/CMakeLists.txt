
function(WrappingMathAddSandboxExecutable)
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs LIB_DEPENDS RESOURCES SOURCE_FILES)
    cmake_parse_arguments(WRAPSAND
            "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    add_executable(${WRAPSAND_NAME} EXCLUDE_FROM_ALL ${WRAPSAND_NAME}.cpp
            ${WRAPSAND_SOURCE_FILES})
    set_target_properties(${WRAPSAND_NAME} PROPERTIES
            FOLDER "Sandbox/WrappingMath")
    target_link_libraries(${WRAPSAND_NAME} ${WRAPSAND_LIB_DEPENDS})
    target_include_directories(${WRAPSAND_NAME} PRIVATE
            ${CMAKE_SOURCE_DIR}/Sandbox/WrappingMath/shared)
    file(COPY ${MOCOSAND_RESOURCES} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
endfunction()

WrappingMathAddSandboxExecutable(NAME sandboxReplaceWrappingMath LIB_DEPENDS osimTools)


