
add_executable(opensim-moco opensim-moco.cpp)
target_link_libraries(opensim-moco osimMoco)

install(TARGETS opensim-moco DESTINATION bin)
set_target_properties(opensim-moco PROPERTIES
        DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
        PROJECT_LABEL "Executable - opensim-moco"
        FOLDER "Moco")


# RPATH
# -----
MocoAddInstallRPATH(TARGET opensim-moco EXECUTABLE
        FROM "${CMAKE_INSTALL_BINDIR}" TO "${CMAKE_INSTALL_LIBDIR}")
MocoAddInstallRPATHSimbody(TARGET opensim-moco EXECUTABLE
        FROM "${CMAKE_INSTALL_BINDIR}")
