if(BUILD_PYTHON_WRAPPING OR BUILD_JAVA_WRAPPING)
    if(${OPENSIM_PYTHON_VERSION} STREQUAL "2")
        set(required_swig_version 3.0.6)
    elseif(${OPENSIM_PYTHON_VERSION} STREQUAL "3")
        # https://github.com/tensorflow/tensorflow/issues/830
        set(required_swig_version 3.0.8)
    endif()
    find_package(SWIG ${required_swig_version} REQUIRED)
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
