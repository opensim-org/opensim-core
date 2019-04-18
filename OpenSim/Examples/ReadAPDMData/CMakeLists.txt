add_executable(readAPDMData readAPDMData.cpp)
target_link_libraries(readAPDMData ${Simbody_LIBRARIES} osimCommon)
set_target_properties(readAPDMData
       PROPERTIES        
       PROJECT_LABEL "Utilities - readAPDMData"
)
if(OPENSIM_BUILD_INDIVIDUAL_APPS)
    install(TARGETS readAPDMData DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif()