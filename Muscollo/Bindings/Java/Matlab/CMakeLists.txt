
find_package(Matlab COMPONENTS MAIN_PROGRAM)
if(Matlab_FOUND AND BUILD_TESTING)
    add_subdirectory(Tests)
endif()
