set(EXAMPLE_2D_WALKING_FILES
        2D_gait.osim
        referenceCoordinates.sto
        referenceGRF.sto
        referenceGRF.xml)

OpenSimAddExampleCXX(NAME example2DWalking
    RESOURCES "${EXAMPLE_2D_WALKING_FILES}")

file(GLOB GEOMETRY "${CMAKE_SOURCE_DIR}/opensim-models/Geometry/*"
        "${CMAKE_SOURCE_DIR}/opensim-models/Models/RajagopalModel/Geometry/*")
file(COPY ${GEOMETRY}
        DESTINATION
        "${CMAKE_BINARY_DIR}/Examples/Moco/example2DWalking/Geometry")
install(FILES ${GEOMETRY}
        DESTINATION "${OPENSIM_INSTALL_CPPEXDIR}/example2DWalking/Geometry")

file(COPY ${EXAMPLE_2D_WALKING_FILES}
        DESTINATION
        "${CMAKE_BINARY_DIR}/Bindings/Java/Matlab/examples/Moco/example2DWalking")
install(FILES ${EXAMPLE_2D_WALKING_FILES}
        DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}/example2DWalking")
install(FILES ${GEOMETRY}
        DESTINATION "${OPENSIM_INSTALL_MATLABEXDIR}/example2DWalking/Geometry")
