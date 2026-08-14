if(OPENSIM_BUILD_INDIVIDUAL_APPS)
    OpenSimAddApplication(NAME analyze)
endif()

if(BUILD_TESTING)
    subdirs(tests)
endif(BUILD_TESTING)
