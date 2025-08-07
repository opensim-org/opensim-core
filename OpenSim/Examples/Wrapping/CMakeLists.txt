find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")
add_executable(exampleScholz2015GeometryPath exampleScholz2015GeometryPath.cpp)
target_link_libraries(exampleScholz2015GeometryPath ${OpenSim_LIBRARIES})
set_target_properties(exampleScholz2015GeometryPath PROPERTIES FOLDER "Examples")
