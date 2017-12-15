# The bindings build on top of OpenSim's bindings. For Java, we create our own
# "org.opensim.modeling" that includes all of opensim-core's SWIG-generated
# .java source files. However, we build a separate native library,
# osimMuscolloJavaJNI.

# We hope that, eventually, the Muscollo code will become part of the
# opensim-core repository, in which case much of the CMake scripting
# surrounding the bindings can be simplified. If Muscollo remains separate,
# though, then we should rename the Java package to "org.opensim.muscollo" and
# rename the Python package to "muscollo".

# Ideally, we would only use the installation of opensim-core. However, we
# must access files within the opensim-core source code repository
# (e.g., OpenSimHeaders_common.h, common.i, java_common.i, python_common.i).
# For this reason, opensim-core is a submodule of the muscollo repository.
# Unfortunately, this makes for a confusing amalgam of files: the Muscollo
# bindings depend on files in both opensim-core's (binary) installation and in
# opensim-core's source code repository.

if(MUSCOLLO_JAVA_BINDINGS OR MUSCOLLO_PYTHON_BINDINGS)
    find_package(SWIG 3.0.8 REQUIRED)
endif()

if(MUSCOLLO_JAVA_BINDINGS)
    if(NOT OpenSim_JAVA_WRAPPING)
        message(SEND_ERROR "If MUSCOLLO_JAVA_BINDINGS=ON, then OpenSim must "
            "have been built with BUILD_JAVA_WRAPPING=ON.")
    endif()
    add_subdirectory(Java)
endif()

if(MUSCOLLO_PYTHON_BINDINGS)
    if(NOT OpenSim_PYTHON_WRAPPING)
        message(SEND_ERROR "If MUSCOLLO_PYTHON_BINDINGS=ON, then OpenSim must "
            "have been built with BUILD_PYTHON_WRAPPING=ON.")
    endif()
    add_subdirectory(Python)
endif()
