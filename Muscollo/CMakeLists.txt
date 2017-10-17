
set(CMAKE_CXX_STANDARD 11)
find_package(OpenSim 4.0 REQUIRED)
set_package_properties(OpenSim PROPERTIES
        URL http://opensim.stanford.edu
        TYPE REQUIRED
        PURPOSE "Musculoskeletal modeling")
# MuscolloCopyDLLs(OpenSim "${OpenSim_ROOT_DIR}/bin")

# Copy OpenSim into our installation.
install(DIRECTORY "${OpenSim_ROOT_DIR}/"
        DESTINATION "${CMAKE_INSTALL_PREFIX}"
        USE_SOURCE_PERMISSIONS)

add_subdirectory(doc)
add_subdirectory(Muscollo)
add_subdirectory(Tests)
add_subdirectory(Executable)
add_subdirectory(Examples)
add_subdirectory(Sandbox)
