/* -------------------------------------------------------------------------- *
 *                     OpenSim:  exampleHopperDevice.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Chris Dembia, Shrinidhi K. Lakshmikanth, Ajay Seth,             *
 *            Thomas Uchida                                                   *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/* This example demonstrates some of the new features of the OpenSim 4.0 API.
The Component architecture allows us to join sub-assemblies to form larger
Models, with information flowing between Components via Inputs, Outputs, and
Sockets. For more information, please refer to the Component documentation.

This interactive example consists of three steps:
  Step 1. Build and simulate a single-legged hopping mechanism.
  Step 2. Build an assistive device and test it on a simple testbed.
  Step 3. Connect the device to the hopper to increase hop height.

To start working through this example, go to run() at the bottom of this file.
From there, you will be directed to specific files and methods in this project
that need to be completed. Now, hop to it! */

#include <OpenSim/OpenSim.h>
#include "defineDeviceAndController_answers.h"

static const double SIGNAL_GEN_CONSTANT{ 0.33 };
static const double REPORTING_INTERVAL{ 0.2 };
static const double FINAL_TIME{ 5.0 };

static const std::string testbedAttachment1{"ground"};
static const std::string testbedAttachment2{"bodyset/load"};

//TODO: Provide the name of the coordinate corresponding to the hopper's height.
//      Hint: the hopper's pelvis is attached to ground with a vertical slider
//      joint; see buildHopperModel.cpp and Component::printOutputInfo().
// [Step 1, Task A]
//static const std::string hopperHeightCoord{"/?????"}; //fill this in
#pragma region Step1_TaskA_solution

static const std::string hopperHeightCoord{"/jointset/slider/yCoord"};

#pragma endregion

//TODO: Provide the absolute path names of the PhysicalOffsetFrames defined on
//      the hopper for attaching the assistive device. See buildHopperModel.cpp
//      and Component::printSubcomponentInfo().
// [Step 3, Task A]
//static const std::string thighAttachment{"/?????"}; //fill this in
//static const std::string shankAttachment{"/?????"}; //fill this in
#pragma region Step3_TaskA_solution

static const std::string thighAttachment{
        "/bodyset/thigh/deviceAttachmentPoint"};
static const std::string shankAttachment{
        "/bodyset/shank/deviceAttachmentPoint"};

#pragma endregion

//TODO: To assist hopping, we will activate the knee device whenever the vastus
//      muscle is active. For convenience, we define a string "vastus" which
//      is the path to the vastus muscle.
// [Step 3, Task B]
//static const std::string vastus{"/?????"}; //fill this in
#pragma region Step3_TaskB_solution

static const std::string vastus{"/forceset/vastus"};

#pragma endregion


namespace OpenSim {

// Forward declarations for methods used below.
Model buildHopper(bool showVisualizer);    //defined in buildHopperModel.cpp
Model buildTestbed(bool showVisualizer);   //defined in buildTestbedModel.cpp
Device* buildDevice();  //defined in buildDevice.cpp


//------------------------------------------------------------------------------
// Attach the device to any two PhysicalFrames in a model.
// [Step 2, Task D]
//------------------------------------------------------------------------------
void connectDeviceToModel(OpenSim::Device& device, OpenSim::Model& model,
    const std::string& modelFrameAname, const std::string& modelFrameBname)
{
    // Add the device to the model.
    model.addComponent(&device);
    
    //TODO: Get writable references to the "anchor" joints in the device.
    #pragma region Step2_TaskD_solution

    auto& anchorA = device.updComponent<WeldJoint>("anchorA");
    auto& anchorB = device.updComponent<WeldJoint>("anchorB");

    #pragma endregion

    //TODO: Recall that the child frame of each anchor (WeldJoint) was attached
    //      to the corresponding cuff. We will now attach the parent frames of
    //      the anchors to modelFrameA and modelFrameB. First get references to
    //      the two specified PhysicalFrames in model (i.e., modelFrameAname and
    //      modelFrameBname), then connect them to the parent frames of each
    //      anchor. (2 lines of code for each anchor.)
    #pragma region Step2_TaskD_solution

    const auto& frameA = model.getComponent<PhysicalFrame>(modelFrameAname);
    anchorA.connectSocket_parent_frame(frameA);
    const auto& frameB = model.getComponent<PhysicalFrame>(modelFrameBname);
    anchorB.connectSocket_parent_frame(frameB);

    #pragma endregion

    // Configure the device to wrap over the patella (if one exists; there is no
    // patella in the testbed).
    const std::string patellaPath("/bodyset/thigh/patellaFrame/wrapobjectset/patella");
    if (model.hasComponent<WrapCylinder>(patellaPath)) {
        auto& cable = model.updComponent<PathActuator>("device/cableAtoB");
        auto& wrapObject = model.updComponent<WrapCylinder>(patellaPath);
        cable.updGeometryPath().addPathWrap(wrapObject);
    }
}


//------------------------------------------------------------------------------
// Add a ConsoleReporter to the hopper model to display variables of interest.
// [Step 1, Task B]
//------------------------------------------------------------------------------
void addConsoleReporterToHopper(Model& hopper)
{
    //TODO: Create a new ConsoleReporter. Set its name and reporting interval.
    #pragma region Step1_TaskB_solution

    auto reporter = new ConsoleReporter();
    reporter->setName("hopper_results");
    reporter->set_report_time_interval(REPORTING_INTERVAL);

    #pragma endregion

    //TODO: Connect outputs from the hopper to the reporter's inputs. Try
    //      reporting the hopper's height, the vastus muscle's activation, the
    //      knee angle, and any other variables of interest.
    #pragma region Step1_TaskB_solution

    reporter->addToReport(
        hopper.getComponent(hopperHeightCoord).getOutput("value"), "height");

    #pragma endregion
    #pragma region Step1_TaskB_solution

    reporter->addToReport(
        hopper.getComponent("/forceset/vastus").getOutput("activation"));

    #pragma endregion
    #pragma region Step1_TaskB_solution

    reporter->addToReport(
        hopper.getComponent("/jointset/knee/kneeFlexion").
            getOutput("value"), "knee_angle");

    #pragma endregion

    //TODO: Add the reporter to the model.
    #pragma region Step1_TaskB_solution

    hopper.addComponent(reporter);

    #pragma endregion
}


//------------------------------------------------------------------------------
// Add a SignalGenerator to a device.
// [Step 2, Task E]
//------------------------------------------------------------------------------
void addSignalGeneratorToDevice(Device& device)
{
    //TODO: Create a new SignalGenerator and set its name.
    #pragma region Step2_TaskE_solution

    auto signalGen = new SignalGenerator();
    signalGen->setName("signalGen");

    #pragma endregion

    // Try changing the constant value and/or the function (e.g., try a
    // LinearFunction).
    signalGen->set_function(Constant(SIGNAL_GEN_CONSTANT));
    device.addComponent(signalGen);

    //TODO: Connect the signal generator's output signal to the controller's
    //      activation input.
    #pragma region Step2_TaskE_solution

    device.updComponent("controller").updInput("activation").connect(
            signalGen->getOutput("signal"));

    #pragma endregion
}


//------------------------------------------------------------------------------
// Add a ConsoleReporter to a model for displaying outputs from a device.
//------------------------------------------------------------------------------
void addDeviceConsoleReporterToModel(Model& model, Device& device,
    const std::vector<std::string>& deviceOutputs,
    const std::vector<std::string>& deviceControllerOutputs)
{
    // Create a new ConsoleReporter. Set its name and reporting interval.
    auto reporter = new ConsoleReporter();
    reporter->setName(model.getName() + "_" + device.getName() + "_results");
    reporter->set_report_time_interval(REPORTING_INTERVAL);

    // Loop through the desired device outputs and add them to the reporter.
    for (auto thisOutputName : deviceOutputs)
        reporter->addToReport(device.getOutput(thisOutputName));

    for (auto thisOutputName : deviceControllerOutputs)
        reporter->addToReport(
                device.getComponent("controller").getOutput(thisOutputName));

    // Add the reporter to the model.
    model.addComponent(reporter);
}

} // namespace OpenSim


void run(bool showVisualizer, double finalTime);

int main(int argc, char* argv[]) {
    //==========================================================================
    // Command line argument parsing.
    //==========================================================================
    // An optional argument to this executable:
    //    noVisualizer -- [Optional] Do not show visualizer. Default
    //                       behavior is to show visualizer.
    
    // Suppress/show visualizer.
    bool showVisualizer{true};

    for(int i = 0; i < argc; ++i)
        if(strcmp(argv[i], "noVisualizer") == 0)
            showVisualizer = false;

    try {
        run(showVisualizer, FINAL_TIME);
    }
    catch (const std::exception& ex) {
        std::cout << "Hopper Example Failed to run due to the following Exception: " 
            << ex.what() << std::endl;
        return 1;
    }

    return 0;
}

//------------------------------------------------------------------------------
// START HERE! Toggle "if (false)" to "if (true)" to enable/disable each step in
// the exercise. The project should execute without making any changes (you
// should see the unassisted hopper hop slightly).
//------------------------------------------------------------------------------


void run(bool showVisualizer, double finalTime)
{
    using namespace OpenSim;

    //==========================================================================
    // Step 1. Build and simulate a single-legged hopping mechanism.
    //==========================================================================
    if (true)
    {
        // Build the hopper.
        auto hopper = buildHopper(showVisualizer);

        // Show all Components in the model.
        hopper.printSubcomponentInfo();
        // Show only the Joints in the model.
        hopper.printSubcomponentInfo<Joint>();
        // Show the outputs generated by the thigh body.
        // The argument indicates if outputs of subcomponents should be printed.
        hopper.getComponent("/bodyset/thigh").printOutputInfo(false);

        // Step 1, Task A
        // ==============
        // Determine the name of the coordinate corresponding to the hopper's
        // height. The hopperHeightCoord string (at the top of this file) must
        // be filled in.

        // Step 1, Task B
        // ==============
        // Report the hopper's height and vastus muscle activation during the
        // simulation. The addConsoleReporterToHopper() method (in this file)
        // needs to be filled in.
        addConsoleReporterToHopper(hopper);

        // Create the system, initialize the state, and simulate.
        SimTK::State& sHop = hopper.initSystem();
        simulate(hopper, sHop, finalTime);
    }

    //==========================================================================
    // Step 2. Build an assistive device and test it on a simple testbed.
    //==========================================================================
    if (true)
    {
        // Build the testbed and device.
        auto testbed = buildTestbed(showVisualizer);

        // Step 2, Task A
        // ==============
        // Go to defineDeviceAndController.h and complete the "TODO"s in the
        // Device class.

        // Step 2, Task B
        // ==============
        // Go to defineDeviceAndController.h and complete the "TODO"s in the
        // PropMyoController class.

        // Step 2, Task C
        // ==============
        // Go to buildDeviceModel.cpp and complete the "TODO"s in buildDevice().
        auto device = buildDevice();

        // Show all Components in the device and testbed.
        device->printSubcomponentInfo();
        testbed.printSubcomponentInfo();
        // Show the outputs generated by the device.
        device->printOutputInfo(false);

        // Step 2, Task D
        // ==============
        // Connect the device to the testbed. The connectDeviceToModel() method
        // (in this file) needs to be filled in.
        connectDeviceToModel(*device, testbed, testbedAttachment1,
                             testbedAttachment2);

        // Step 2, Task E
        // ==============
        // Use a SignalGenerator to create a control signal for testing the
        // device. The addSignalGeneratorToDevice() method (in this file) needs
        // to be filled in.
        addSignalGeneratorToDevice(*device);

        // List the device outputs we wish to display during the simulation.
        std::vector<std::string> deviceOutputs{ "length", "tension", "power" };
        std::vector<std::string> deviceControllerOutputs{ "myo_control" };

        // Add a ConsoleReporter to report deviceOutputs.
        addDeviceConsoleReporterToModel(testbed, *device, deviceOutputs,
            deviceControllerOutputs);

        // Create the system, initialize the state, and simulate.
        SimTK::State& sDev = testbed.initSystem();
        simulate(testbed, sDev, finalTime);
    }

    //==========================================================================
    // Step 3. Connect the device to the hopper to increase hop height.
    //==========================================================================
    if (true)
    {
        // Build the hopper and device.
        auto assistedHopper = buildHopper(showVisualizer);
        auto kneeDevice = buildDevice();

        // Step 3, Task A
        // ==============
        // Connect the device to the hopper. The thighAttachment and
        // shankAttachment strings (at the top of this file) must be filled in.
        connectDeviceToModel(*kneeDevice, assistedHopper, thighAttachment,
                             shankAttachment);

        // Step 3, Task B
        // ==============
        // Use the vastus muscle's activation as the control signal for the
        // device. The vastus string (at the top of this file) must
        // be filled in.
        kneeDevice->updComponent("controller").updInput("activation").connect(
                assistedHopper.getComponent(vastus).getOutput("activation"));

        // List the device outputs we wish to display during the simulation.
        std::vector<std::string> kneeDeviceOutputs{ "tension", "height" };
        std::vector<std::string> deviceControllerOutputs{ "myo_control" };

        // Add a ConsoleReporter to report deviceOutputs.
        addDeviceConsoleReporterToModel(assistedHopper, *kneeDevice,
                                        kneeDeviceOutputs, deviceControllerOutputs);

        // Create the system, initialize the state, and simulate.
        SimTK::State& sHD = assistedHopper.initSystem();
        simulate(assistedHopper, sHD, finalTime);
    }
};
