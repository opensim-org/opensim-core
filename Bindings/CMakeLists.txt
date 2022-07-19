if(BUILD_PYTHON_WRAPPING OR BUILD_JAVA_WRAPPING)
    find_package(SWIG 4.0.2 REQUIRED)
endif()

# Flags are both Python and Java bindings will use.
if(WITH_EZC3D)
    set(SWIG_FLAGS "-DWITH_EZC3D")
endif()
#flag to indicate whether to let SWIG process Doxygen comments
# Will be on for releases but too many warnings/errors for regular dev.
set(SWIG_DOXYGEN ON CACHE BOOL "Carry Doxygen comments to bindings")
mark_as_advanced(SWIG_DOXYGEN)
set(SWIG_DOXYGEN_STRING "")
if (SWIG_DOXYGEN)
    set(SWIG_DOXYGEN_STRING "-doxygen")
endif()

if(BUILD_PYTHON_WRAPPING)
    add_subdirectory(Python)
endif()
if(BUILD_JAVA_WRAPPING)
    add_subdirectory(Java)
endif()
