include_directories(${PROJECT_SOURCE_DIR})

add_executable(exampleHopperDevice
    exampleHopperDevice.cpp
    buildHopperModel.cpp
    buildDeviceModel.cpp
    defineDeviceAndController.h
    helperMethods.h)
target_link_libraries(exampleHopperDevice osimTools)
set_target_properties(exampleHopperDevice PROPERTIES FOLDER "Examples")

add_executable(exampleHopperDeviceAnswers
    exampleHopperDevice_answers.cpp
    buildHopperModel.cpp
    buildDeviceModel_answers.cpp
    defineDeviceAndController_answers.h
    helperMethods.h)
target_link_libraries(exampleHopperDeviceAnswers osimTools)
set_target_properties(exampleHopperDeviceAnswers PROPERTIES FOLDER "Examples")
