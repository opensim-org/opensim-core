
if(Matlab_FOUND)
    if(BUILD_TESTING)
        add_subdirectory(tests)
    endif()
endif()

# Install.
# --------
install(DIRECTORY examples/ DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}")
install(DIRECTORY Hopper_Device DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}")

# Remove parts from the hopper device example that the tutorial audience is
# expected to fill in.
# Anything matching `% ANSWER{ ... % }` is cut from the file.
foreach(fname RunHopper RunHopperWithDevice)
    file(STRINGS Hopper_Device/${fname}_answers.m file_contents_answers)
    list(LENGTH file_contents_answers num_lines)
    set(line_num 0)
    while(${line_num} LESS ${num_lines})
        list(GET file_contents_answers ${line_num} current_line)
        if("${current_line}" MATCHES "% ANSWER{")
            while(NOT "${current_line}" MATCHES "% }")
                math(EXPR line_num "${line_num} + 1")
                list(GET file_contents_answers ${line_num} current_line)
            endwhile()
            # Add an empty line in place of the ANSWERS block.
            set(file_contents "${file_contents}\n")
        else()
            set(file_contents "${file_contents}${current_line}\n")
        endif()
        math(EXPR line_num "${line_num} + 1")
    endwhile()
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/Hopper_Device/${fname}.m"
        "${file_contents}")
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/Hopper_Device/${fname}.m"
        DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}/Hopper_Device")
endforeach()

# This script contains paths into the OpenSim installation that may be
# different on different platforms, so we configure it with CMake variables.
configure_file(configureOpenSim.m.in
    "${CMAKE_CURRENT_BINARY_DIR}/configureOpenSim.m" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/configureOpenSim.m"
        DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}")
