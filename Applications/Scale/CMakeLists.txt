if(OPENSIM_BUILD_INDIVIDUAL_APPS)
    OpenSimAddApplication(NAME scale)
endif()

if(BUILD_TESTING)
    subdirs(test)
endif(BUILD_TESTING)
