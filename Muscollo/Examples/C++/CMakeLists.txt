add_subdirectory(exampleSlidingMass)

install(DIRECTORY exampleSlidingMass
        DESTINATION ${MUSCOLLO_INSTALL_EXAMPLEDIR}/C++
        PATTERN "CMakeLists.txt" EXCLUDE)

set(_example_install_dir ${MUSCOLLO_INSTALL_EXAMPLEDIR}/C++/exampleSlidingMass)
set(_example_name exampleSlidingMass) # Used inside the .in file.
file(RELATIVE_PATH _muscollo_install_hint
        "${CMAKE_INSTALL_PREFIX}/${_example_install_dir}"
        "${CMAKE_INSTALL_PREFIX}")
configure_file(CMakeListsToInstall.txt.in
    "${CMAKE_CURRENT_BINARY_DIR}/exampleSlidingMass/CMakeListsToInstall.txt"
        @ONLY)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/exampleSlidingMass/CMakeListsToInstall.txt"
        DESTINATION ${_example_install_dir}
        RENAME CMakeLists.txt)
