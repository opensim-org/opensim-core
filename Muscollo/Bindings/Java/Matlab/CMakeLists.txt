
find_package(Matlab 7.0 COMPONENTS MAIN_PROGRAM)
if(Matlab_FOUND AND BUILD_TESTING)
    add_subdirectory(Tests)
endif()
