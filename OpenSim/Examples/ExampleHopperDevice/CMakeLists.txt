add_executable(exampleHopperDevice
    exampleHopperDevice.cpp
    buildHopperModel.cpp
    buildDeviceModel.cpp
    buildTestbedModel.cpp
    defineDeviceAndController.h)
target_link_libraries(exampleHopperDevice ${Simbody_LIBRARIES} osimTools)
set_target_properties(exampleHopperDevice PROPERTIES FOLDER "Examples")
add_test(NAME    exampleHopperDevice
         COMMAND exampleHopperDevice noVisualizer simulateOnce)

add_executable(exampleHopperDeviceAnswers
    exampleHopperDevice_answers.cpp
    buildHopperModel.cpp
    buildTestbedModel.cpp
    buildDeviceModel_answers.cpp
    defineDeviceAndController_answers.h)
target_link_libraries(exampleHopperDeviceAnswers ${Simbody_LIBRARIES} osimTools)
set_target_properties(exampleHopperDeviceAnswers PROPERTIES FOLDER "Examples")
# The answers file uses `#pragma region` to facilitate code folding
# when using this example in a live demonstration, but these pragmas are only
# recognized by MSVC.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(exampleHopperDeviceAnswers PUBLIC
            -Wno-unknown-pragmas)
endif()
add_test(NAME    exampleHopperDeviceAnswers
         COMMAND exampleHopperDeviceAnswers noVisualizer)
