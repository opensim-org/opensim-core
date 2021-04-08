if(BUILD_PYTHON_WRAPPING OR BUILD_JAVA_WRAPPING)
    # We require 3.0.8 for the reason mentioned here (for Python 3):
    # https://github.com/tensorflow/tensorflow/issues/830
    find_package(SWIG 4.0.0 REQUIRED)
endif()

# Flags are both Python and Java bindings will use.
if(WITH_BTK)
    set(SWIG_FLAGS "-DWITH_BTK")
endif()
if(WITH_EZC3D)
    set(SWIG_FLAGS "-DWITH_EZC3D")
endif()

if(BUILD_PYTHON_WRAPPING)
    add_subdirectory(Python)
endif()
if(BUILD_JAVA_WRAPPING)
    add_subdirectory(Java)
endif()
