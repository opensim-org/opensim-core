add_executable(exampleHopperDevice exampleHopperDevice.cpp buildHopperModel.cpp buildDeviceModel.cpp defineDeviceAndController.h helperMethods.h)
target_link_libraries(exampleHopperDevice osimTools)
set_target_properties(exampleHopperDevice PROPERTIES
    PROJECT_LABEL "Hopper Device"
    FOLDER "Examples"
    EXCLUDE_FROM_ALL TRUE
    )
