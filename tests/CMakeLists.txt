find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

if(BUILD_TESTING)
    add_library(osimTesting STATIC Testing.cpp Testing.h)
    OpenSimConfigureTarget(osimTesting)
    target_link_libraries(osimTesting PUBLIC osimCommon Catch2::Catch2)
endif()
