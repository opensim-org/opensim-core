if(BUILD_PYTHON_WRAPPING OR BUILD_JAVA_WRAPPING)
    find_package(SWIG 3.0.6 REQUIRED)
endif()

# Flags are both Python and Java bindings will use.
if(WITH_BTK)
    set(SWIG_FLAGS "-DWITH_BTK")
endif()

if(BUILD_PYTHON_WRAPPING)
    add_subdirectory(Python)
endif()
if(BUILD_JAVA_WRAPPING)
    add_subdirectory(Java)
endif()
