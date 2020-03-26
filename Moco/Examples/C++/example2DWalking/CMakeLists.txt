set(EXAMPLE_2D_WALKING_FILES
        2D_gait.osim
        referenceCoordinates.sto
        referenceGRF.sto
        referenceGRF.xml)

MocoAddExampleCXX(NAME example2DWalking
                  RESOURCES "${EXAMPLE_2D_WALKING_FILES}")

file(GLOB GEOMETRY "${CMAKE_SOURCE_DIR}/opensim-models/Geometry/*"
        "${CMAKE_SOURCE_DIR}/opensim-models/Models/RajagopalModel/Geometry/*")
file(COPY ${GEOMETRY}
        DESTINATION
        "${CMAKE_BINARY_DIR}/Moco/Examples/C++/example2DWalking/Geometry")
install(FILES ${GEOMETRY}
        DESTINATION "${MOCO_INSTALL_CPPEXDIR}/example2DWalking/Geometry")

file(COPY ${EXAMPLE_2D_WALKING_FILES}
        DESTINATION
        "${CMAKE_BINARY_DIR}/Moco/Bindings/Java/Matlab/Examples/example2DWalking")
install(FILES ${EXAMPLE_2D_WALKING_FILES}
        DESTINATION "${MOCO_INSTALL_MATLABEXDIR}/example2DWalking")
install(FILES ${GEOMETRY}
        DESTINATION "${MOCO_INSTALL_MATLABEXDIR}/example2DWalking/Geometry")
