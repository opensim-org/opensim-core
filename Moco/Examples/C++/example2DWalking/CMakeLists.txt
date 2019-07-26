set(EXAMPLE_MOCO_2D_WALKING_FILES
      gait10dof18musc.osim
      referenceCoordinates.sto)

MocoAddExampleCXX(NAME example2DWalking
                  RESOURCES "${EXAMPLE_MOCO_2D_WALKING_FILES}")

file(GLOB GEOMETRY "${CMAKE_SOURCE_DIR}/opensim-models/Geometry/*"
        "${CMAKE_SOURCE_DIR}/opensim-models/Models/RajagopalModel/Geometry/*")
file(COPY ${GEOMETRY}
        DESTINATION
        "${CMAKE_BINARY_DIR}/Moco/Examples/C++/example2DWalking/Geometry")
install(FILES ${GEOMETRY}
        DESTINATION "${MOCO_INSTALL_CPPEXDIR}/example2DWalking/Geometry")
