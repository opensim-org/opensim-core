
if(Matlab_FOUND)
    if(BUILD_TESTING)
        add_subdirectory(tests)
    endif()
endif()

# Install.
# --------
install(DIRECTORY examples/ DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}")
install(DIRECTORY Hopper_Device DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}")
install(DIRECTORY Utilities DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}")
install(DIRECTORY OpenSenseExample DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}")
install(DIRECTORY Dynamic_Walking_Tutorials/Dynamic_Walker_Builder
        DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}")
install(DIRECTORY Dynamic_Walking_Tutorials/Dynamic_Walker_Challenge
        DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}"
        # These files have not yet been updated for 4.0:
        PATTERN "DesignMainStarterWithControls.m" EXCLUDE
        PATTERN "AddCoordinateActuator.m" EXCLUDE
        PATTERN "AddMillardMuscle.m" EXCLUDE
        PATTERN "AddPathActuator.m" EXCLUDE
        PATTERN "IntegrateOpenSimPlant.m" EXCLUDE
        PATTERN "Main_WalkerForwardSim.m" EXCLUDE
        PATTERN "Main_WalkerForwardSimWithControls.m" EXCLUDE
        PATTERN "OpenSimPlantControlsFunction.m" EXCLUDE
        PATTERN "OpenSimPlantFunction.m" EXCLUDE
        PATTERN "Simulation_using_Matlab_Integration.m" EXCLUDE)
install(DIRECTORY CustomStaticOptimization DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}")

# Remove parts from the hopper device example that the tutorial audience is
# expected to fill in.
# Anything matching `% ANSWER{ ... % }` is cut from the file.
foreach(fname RunHopper RunHopperWithDevice)
    set(file_contents "")
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
    install(FILES       "${CMAKE_CURRENT_BINARY_DIR}/Hopper_Device/${fname}.m"
            DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}/Hopper_Device")
endforeach()

# Copy example model and data files to use in Examples.
file(COPY "examples/Moco/exampleSquatToStand/squatToStand_3dof9musc.osim"
        DESTINATION
        "${CMAKE_BINARY_DIR}/Examples/Moco/exampleSquatToStand")

# Copy and install the example2DWalking resources from the C++ example.
set(EXAMPLE_2D_WALKING_DIR
    "${CMAKE_SOURCE_DIR}/OpenSim/Examples/Moco/example2DWalking")
set(EXAMPLE_2D_WALKING_FILES
    "${EXAMPLE_2D_WALKING_DIR}/2D_gait.osim"
    "${EXAMPLE_2D_WALKING_DIR}/referenceCoordinates.sto"
    "${EXAMPLE_2D_WALKING_DIR}/referenceGRF.sto"
    "${EXAMPLE_2D_WALKING_DIR}/referenceGRF.xml")

file(COPY ${EXAMPLE_2D_WALKING_FILES}
     DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/examples/Moco/example2DWalking")
install(FILES ${EXAMPLE_2D_WALKING_FILES}
        DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}/Moco/example2DWalking")

# Copy and install the example3DWalking resources from the C++ example.
set(EXAMPLE_3D_WALKING_DIR
    "${CMAKE_SOURCE_DIR}/OpenSim/Examples/Moco/example3DWalking")
set(EXAMPLE_MOCO_TRACK_FILES
    "${EXAMPLE_3D_WALKING_DIR}/subject_walk_scaled.osim"
    "${EXAMPLE_3D_WALKING_DIR}/subject_walk_scaled_FunctionBasedPathSet.xml"
    "${EXAMPLE_3D_WALKING_DIR}/subject_walk_scaled_ContactForceSet.xml"
    "${EXAMPLE_3D_WALKING_DIR}/subject_walk_scaled_ContactGeometrySet.xml"
    "${EXAMPLE_3D_WALKING_DIR}/subject_walk_scaled_ExpressionBasedCoordinateForceSet.xml"
    "${EXAMPLE_3D_WALKING_DIR}/coordinates.sto"
    "${EXAMPLE_3D_WALKING_DIR}/grf_walk.mot"
    "${EXAMPLE_3D_WALKING_DIR}/grf_walk.xml"
    "${EXAMPLE_3D_WALKING_DIR}/markers_walk.trc"
    "${EXAMPLE_3D_WALKING_DIR}/ik_tasks_walk.xml"
    "${EXAMPLE_3D_WALKING_DIR}/electromyography.sto"
    "${EXAMPLE_3D_WALKING_DIR}/id_walk.sto"
    "${EXAMPLE_3D_WALKING_DIR}/README.txt")

# Copy and install the exampleMocoTrack resources from the C++ example.
file(COPY ${EXAMPLE_MOCO_TRACK_FILES}
     DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/examples/Moco/example3DWalking")
install(FILES ${EXAMPLE_MOCO_TRACK_FILES}
        DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}/Moco/example3DWalking")

# Copy and install the examplePolynomialPathFitter resources from the C++ example.
set(EXAMPLE_PATH_FITTER_FILES
    "${EXAMPLE_3D_WALKING_DIR}/subject_walk_scaled.osim"
    "${EXAMPLE_3D_WALKING_DIR}/coordinates.sto")

file(COPY ${EXAMPLE_PATH_FITTER_FILES}
     DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/examples/PolynomialPathFitter")
install(FILES ${EXAMPLE_PATH_FITTER_FILES}
        DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}/PolynomialPathFitter")

# Copy Geometry folder for squat-to-stand examples
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/OpenSenseExample/Geometry/"
        DESTINATION
        "${OPENSIM_INSTALL_MATLABEXDIR}/Moco/exampleSquatToStand/Geometry/")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/OpenSenseExample/Geometry/"
        DESTINATION
        "${OPENSIM_INSTALL_PYTHONEXDIR}/Moco/exampleSquatToStand/Geometry/")

# Copy Geometry folder for Moco examples
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/OpenSenseExample/Geometry/"
        DESTINATION
        "${OPENSIM_INSTALL_MATLABEXDIR}/Moco/example2DWalking/Geometry/")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/OpenSenseExample/Geometry/"
        DESTINATION
        "${OPENSIM_INSTALL_MATLABEXDIR}/Moco/example3DWalking/Geometry/")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/OpenSenseExample/Geometry/"
        DESTINATION
        "${OPENSIM_INSTALL_MATLABEXDIR}/Moco/exampleEMGTracking/Geometry/")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/OpenSenseExample/Geometry/"
        DESTINATION
        "${OPENSIM_INSTALL_MATLABEXDIR}/Moco/exampleMarkerTracking10DOF/Geometry/")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/OpenSenseExample/Geometry/"
        DESTINATION
        "${OPENSIM_INSTALL_MATLABEXDIR}/Moco/exampleSquatToStand/Geometry/")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/OpenSenseExample/Geometry/"
        DESTINATION
        "${OPENSIM_INSTALL_MATLABEXDIR}/Moco/exampleSquatToStand/exampleIMUTracking/Geometry/")

# Copy Geometry folder for PolynomialPathFitter examples
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/OpenSenseExample/Geometry/"
        DESTINATION
        "${OPENSIM_INSTALL_MATLABEXDIR}/PolynomialPathFitter/Geometry/")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/OpenSenseExample/Geometry/"
        DESTINATION
        "${OPENSIM_INSTALL_PYTHONEXDIR}/PolynomialPathFitter/Geometry/")

# The configureOpenSim.m script contains paths into the OpenSim installation
# that may be different on different platforms, so we configure it with CMake
# variables.
file(RELATIVE_PATH matlabexdir_to_install_dir
    "${CMAKE_INSTALL_PREFIX}/${OPENSIM_INSTALL_MATLABEXDIR}"
    "${CMAKE_INSTALL_PREFIX}")
configure_file(configureOpenSim.m.in
    "${CMAKE_CURRENT_BINARY_DIR}/configureOpenSim.m" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/configureOpenSim.m"
        DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}")
