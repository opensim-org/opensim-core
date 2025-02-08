MocoAddSandboxExecutable(NAME sandboxSquatToStand
        LIB_DEPENDS osimMoco)

file(COPY "${CMAKE_SOURCE_DIR}/Bindings/Java/Matlab/examples/Moco/exampleSquatToStand/squatToStand_3dof9musc.osim"
        DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
