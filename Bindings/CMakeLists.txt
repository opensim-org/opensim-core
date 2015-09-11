
if(BUILD_PYTHON_WRAPPING)
    find_package(SWIG 3.0.5 REQUIRED)
	add_subdirectory(Python)
endif()
if(BUILD_PYTHON_WRAPPING)
    find_package(SWIG 3.0.5 REQUIRED)
	add_subdirectory(Java)
endif()

