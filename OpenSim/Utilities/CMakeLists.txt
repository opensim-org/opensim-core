set(BUILD_SIMM_TRANSLATOR OFF CACHE BOOL "Build Utilities to support importing/exporting SIMM models")

if(BUILD_JAVA_WRAPPING)
    subdirs(simmFileWriterDLL) 
endif(BUILD_JAVA_WRAPPING)
if(BUILD_SIMM_TRANSLATOR)
    subdirs(simmToOpenSim) 
endif()
