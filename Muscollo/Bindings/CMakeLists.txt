# The bindings build on top of OpenSim's bindings. For Java, we create our own
# "org.opensim.modeling" that includes all of opensim-core's SWIG-generated
# .java source files. However, we build a separate native library,
# osimMuscolloJavaJNI.

if(MUSCOLLO_JAVA_BINDINGS)
    find_package(SWIG 3.0.8 REQUIRED)
endif()

if(MUSCOLLO_JAVA_BINDINGS)
    add_subdirectory(Java)
endif()
