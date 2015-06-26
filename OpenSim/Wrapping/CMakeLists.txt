
if(BUILD_PYTHON_WRAPPING OR BUILD_JAVA_WRAPPING)
    find_package(SWIG 2.0.9 REQUIRED)
endif()

subdirs(Java Python)
