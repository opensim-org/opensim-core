add_executable(readXsensData readXsensData.cpp)
target_link_libraries(readXsensData ${Simbody_LIBRARIES} osimCommon)
set_target_properties(readXsensData
       PROPERTIES        
       PROJECT_LABEL "Utilities - readXsensData"
)
if(OPENSIM_BUILD_INDIVIDUAL_APPS)
    install(TARGETS readXsensData DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif()