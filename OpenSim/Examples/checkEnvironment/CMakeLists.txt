project(Test)

cmake_minimum_required(VERSION 3.2)

# Settings.
# ---------
set(TARGET checkEnvironment CACHE STRING "Name of example to build")

# OpenSim uses c++20 language features.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find and hook up to OpenSim.
# ----------------------------
find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

# Configure this project.
# -----------------------
set(SOURCE_FILES ${TARGET}.cpp)

add_executable(${TARGET}  ${SOURCE_FILES})

target_link_libraries(${TARGET} ${OpenSim_LIBRARIES})

set_target_properties(
    ${TARGET} PROPERTIES
    PROJECT_LABEL "Test Environment")


# Configure installation.
# -----------------------
install_targets(/bin ${TARGET})
