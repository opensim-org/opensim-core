
if(BUILD_PYTHON_WRAPPING OR BUILD_JAVA_WRAPPING)
    find_package(SWIG 3.0.6 REQUIRED)
endif()

subdirs(Java Python)
