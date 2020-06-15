
set(CMAKE_CXX_STANDARD 11)

find_package(OpenSim 4.1 REQUIRED)
set_package_properties(OpenSim PROPERTIES
        URL http://opensim.stanford.edu
        TYPE REQUIRED
        PURPOSE "Musculoskeletal modeling")
MocoCopyDLLs(DEP_NAME OpenSim
    DEP_BIN_DIR "${OpenSim_ROOT_DIR}/bin")

include("${CMAKE_SOURCE_DIR}/opensim-core/cmake/OpenSimMacros.cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/opensim-core/cmake")

# Copy OpenSim into our installation.
install(DIRECTORY "${OpenSim_ROOT_DIR}/"
        DESTINATION "${CMAKE_INSTALL_PREFIX}"
        USE_SOURCE_PERMISSIONS)

add_subdirectory(doc)
add_subdirectory(Moco)
if(BUILD_TESTING)
    add_subdirectory(Tests)
endif()
if(MOCO_BUILD_EXECUTABLE)
    add_subdirectory(Executable)
endif()
add_subdirectory(Bindings)
if(MOCO_BUILD_EXAMPLES)
    add_subdirectory(Examples)
endif()
add_subdirectory(Sandbox)
