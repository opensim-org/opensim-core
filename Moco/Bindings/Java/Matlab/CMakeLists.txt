
find_package(Matlab COMPONENTS MAIN_PROGRAM)
if(Matlab_FOUND AND BUILD_TESTING)
    add_subdirectory(Tests)
endif()

install(DIRECTORY Examples/ DESTINATION "${MOCO_INSTALL_MATLABEXDIR}")
install(DIRECTORY Utilities/
        DESTINATION "${MOCO_INSTALL_MATLABEXDIR}/Utilities")

# Copy example model and data files to use in Sandbox.
set(SANDBOX_MARKER_TRACKING_DIR 
    "${CMAKE_BINARY_DIR}/Moco/Sandbox/sandboxMarkerTrackingWholeBody")
file(COPY "Examples/exampleMarkerTracking10DOF/marker_trajectories.trc"
          "Examples/exampleMarkerTracking10DOF/subject01.osim"
          "Examples/exampleMarkerTracking10DOF/state_reference.mot"
     DESTINATION "${SANDBOX_MARKER_TRACKING_DIR}")
file(COPY "Examples/exampleSquatToStand/squatToStand_3dof9musc.osim"
     DESTINATION
        "${CMAKE_BINARY_DIR}/Moco/Sandbox/sandboxSquatToStand")

# Copy example model and data files to use in Examples.
file(COPY "Examples/exampleSquatToStand/squatToStand_3dof9musc.osim"
     DESTINATION
        "${CMAKE_BINARY_DIR}/Moco/Examples/C++/exampleSquatToStand")

file(GLOB GEOMETRY "${CMAKE_SOURCE_DIR}/opensim-models/Geometry/*"
    "${CMAKE_SOURCE_DIR}/opensim-models/Models/RajagopalModel/Geometry/*")
install(FILES ${GEOMETRY}
    DESTINATION "${MOCO_INSTALL_MATLABEXDIR}/exampleSquatToStand/Geometry")
install(FILES ${GEOMETRY}
    DESTINATION "${MOCO_INSTALL_MATLABEXDIR}/exampleMarkerTracking10DOF/Geometry")

# The configureMoco.m script contains paths into the Moco installation
# that may be different on different platforms, so we configure it with CMake
# variables.
file(RELATIVE_PATH matlabexdir_to_install_dir
    "${CMAKE_INSTALL_PREFIX}/${MOCO_INSTALL_MATLABEXDIR}"
    "${CMAKE_INSTALL_PREFIX}")
configure_file(configureMoco.m.in
    "${CMAKE_CURRENT_BINARY_DIR}/configureMoco.m" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/configureMoco.m"
        DESTINATION "${MOCO_INSTALL_MATLABEXDIR}")
