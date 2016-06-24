add_executable(exampleHopperDevice
    exampleHopperDevice.cpp
    buildHopperModel.cpp
    buildDeviceModel.cpp
    defineDeviceAndController.h
    helperMethods.h)
target_link_libraries(exampleHopperDevice osimTools)
set_target_properties(exampleHopperDevice PROPERTIES
    PROJECT_LABEL "Hopper Device"
    FOLDER "Examples"
    EXCLUDE_FROM_ALL TRUE
    )

add_executable(exampleHopperDeviceAnswers
    exampleHopperDevice_answers.cpp
    buildHopperModel.cpp
    buildDeviceModel_answers.cpp
    defineDeviceAndController_answers.h
    helperMethods.h)
target_link_libraries(exampleHopperDeviceAnswers osimTools)
set_target_properties(exampleHopperDeviceAnswers PROPERTIES
    PROJECT_LABEL "Hopper Device Answers"
    FOLDER "Example answers"
    EXCLUDE_FROM_ALL TRUE
    )
