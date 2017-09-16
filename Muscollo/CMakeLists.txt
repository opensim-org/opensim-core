
set(CMAKE_CXX_STANDARD 11)
find_package(OpenSim 4.0 REQUIRED)
MuscolloCopyDLLs(OpenSim "${OpenSim_ROOT_DIR}/bin")

add_subdirectory(doc)
add_subdirectory(Muscollo)
add_subdirectory(tests)
add_subdirectory(executable)
add_subdirectory(sandbox)
