if(MUSCOLLO_JAVA_BINDINGS)
    find_package(SWIG 3.0.8 REQUIRED)
endif()

if(MUSCOLLO_JAVA_BINDINGS)
    add_subdirectory(Java)
endif()
