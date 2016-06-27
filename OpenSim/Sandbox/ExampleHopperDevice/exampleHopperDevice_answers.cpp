/* -------------------------------------------------------------------------- *
 *                     OpenSim:  exampleHopperDevice.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
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
Connectors. For more information, please refer to the Component documentation.

This interactive example consists of three steps:
  Step 1. Build and simulate a single-legged hopping mechanism.
  Step 2. Build an assistive device and test it on a simple testbed.
  Step 3. Connect the device to the hopper to increase hop height.

To start working through this example, go to main() at the bottom of this file.
From there, you will be directed to specific files and methods in this project
that need to be completed. Now, hop to it! */

#include <OpenSim/OpenSim.h>
#include "defineDeviceAndController_answers.h"
#include "helperMethods.h"

static const double SIGNAL_GEN_CONSTANT{ 0.33 };
static const double REPORTING_INTERVAL{ 0.2 };

static const std::string testbedAttachment1{"ground"};
static const std::string testbedAttachment2{"load"};

//TODO: Provide the name of the output corresponding to the hopper's height.
//      Hint: the hopper's pelvis is attached to ground with a vertical slider
//      joint; see buildHopperModel.cpp and showAllOutputs() in helperMethods.h.
// [Step 1, Task A]
//static const std::string hopperHeightOutput{"/Dennis/?????"}; //fill this in
static const std::string hopperHeightOutput{"/Dennis/slider/height/value"};

//TODO: Provide the full path names of the PhysicalOffsetFrames defined on the
//      hopper for attaching the assistive device. See buildHopperModel.cpp and
//      showSubcomponentInfo() in helperMethods.h.
// [Step 3, Task A]
//static const std::string thighAttachment{"/Dennis/?????"}; //fill this in
//static const std::string shankAttachment{"/Dennis/?????"}; //fill this in
static const std::string thighAttachment{"/Dennis/thigh/deviceAttachmentPoint"};
static const std::string shankAttachment{"/Dennis/shank/deviceAttachmentPoint"};

//TODO: To assist hopping, we will activate the knee device whenever the vastus
//      muscle is active. To do this, we will need to connect the vastus
//      muscle's "activation" output to the controller's "activation" input.
// [Step 3, Task B]
//static const std::string vastusActivationOutput{"/Dennis/?????"}; //fill this in
static const std::string vastusActivationOutput{"/Dennis/vastus/activation"};


namespace OpenSim {

// Forward declarations for methods used below.
Model buildHopper();    //defined in buildHopperModel.cpp
Model buildTestbed();   //defined in helperMethods.h
Device* buildDevice();  //defined in buildDevice.cpp


//------------------------------------------------------------------------------
// Attach the device to any two PhysicalFrames in a model.
// [Step 2, Task D]
//------------------------------------------------------------------------------
void connectDeviceToModel(OpenSim::Device& device, OpenSim::Model& model,
    const std::string& modelFrameAname, const std::string& modelFrameBname)
{
    //TODO: Get writable references to the "anchor" joints in the device.
    auto& anchorA = device.updComponent<Joint>("anchorA");
    auto& anchorB = device.updComponent<Joint>("anchorB");

    //TODO: Recall that the child frame of each anchor (WeldJoint) was attached
    //      to the corresponding cuff. We will now attach the parent frames of
    //      the anchors to modelFrameA and modelFrameB. First get references to
    //      the two specified PhysicalFrames in model (i.e., modelFrameAname and
    //      modelFrameBname), then connect them to the parent frames of each
    //      anchor. (2 lines of code for each anchor.)
    const auto& frameA = model.getComponent<PhysicalFrame>(modelFrameAname);
    anchorA.updConnector("parent_frame").connect(frameA);
    const auto& frameB = model.getComponent<PhysicalFrame>(modelFrameBname);
    anchorB.updConnector("parent_frame").connect(frameB);

    // Add the device to the model. We need to add the device using
    // addModelComponent() rather than addComponent() because of a bug in
    // Model::initSystem().
    model.addModelComponent(&device);

    // Configure the device to wrap over the patella (if one exists; there is no
    // patella in the testbed).
    if (model.hasComponent<WrapCylinder>("thigh/patella")) {
        auto& cable = model.updComponent<PathActuator>("device/cableAtoB");
        auto& frame = model.updComponent<PhysicalFrame>("thigh");
        auto& wrapObject = frame.upd_WrapObjectSet().get("patella");
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
    auto reporter = new ConsoleReporter();
    reporter->setName("hopper_results");
    reporter->set_report_time_interval(REPORTING_INTERVAL);

    //TODO: Connect outputs from the hopper to the reporter's inputs. Try
    //      reporting the hopper's height, the vastus muscle's activation, the
    //      knee angle, and any other variables of interest.
    reporter->updInput("inputs").connect(
        hopper.getOutput(hopperHeightOutput), "height");
    reporter->updInput("inputs").connect(
        hopper.getOutput("/Dennis/vastus/activation"));
    reporter->updInput("inputs").connect(
        hopper.getOutput("/Dennis/knee/kneeFlexion/value"), "knee_angle");

    //TODO: Add the reporter to the model.
    hopper.addComponent(reporter);
}


//------------------------------------------------------------------------------
// Add a SignalGenerator to a device.
// [Step 2, Task E]
//------------------------------------------------------------------------------
void addSignalGeneratorToDevice(Device& device)
{
    //TODO: Create a new SignalGenerator and set its name.
    auto signalGen = new SignalGenerator();
    signalGen->setName("signalGen");

    // Try changing the constant value and/or the function (e.g., try a
    // LinearFunction).
    signalGen->set_function(Constant(SIGNAL_GEN_CONSTANT));
    device.addComponent(signalGen);

    //TODO: Connect the signal generator's output signal to the controller's
    //      activation input ("controller/activation").
    device.updInput("controller/activation")
        .connect(signalGen->getOutput("signal"));
}


//------------------------------------------------------------------------------
// Add a ConsoleReporter to a model for displaying outputs from a device.
//------------------------------------------------------------------------------
void addDeviceConsoleReporterToModel(Model& model, Device& device,
    const std::vector<std::string>& deviceOutputs)
{
    // Create a new ConsoleReporter. Set its name and reporting interval.
    auto reporter = new ConsoleReporter();
    reporter->setName(model.getName() + "_" + device.getName() + "_results");
    reporter->set_report_time_interval(REPORTING_INTERVAL);

    // Loop through the desired device outputs and add them to the reporter.
    for (auto thisOutputName : deviceOutputs)
        reporter->updInput("inputs").connect(device.getOutput(thisOutputName));

    // Add the reporter to the model.
    model.addComponent(reporter);
}

} // namespace OpenSim


//------------------------------------------------------------------------------
// START HERE! Toggle "if (false)" to "if (true)" to enable/disable each step in
// the exercise. The project should execute without making any changes (you
// should see the unassisted hopper hop slightly).
//------------------------------------------------------------------------------
int main()
{
    using namespace OpenSim;

    //==========================================================================
    // Step 1. Build and simulate a single-legged hopping mechanism.
    //==========================================================================
    if (true)
    {
        // Build the hopper.
        auto hopper = buildHopper();
        // Update the hopper model's internal data members, which includes
        // identifying its subcomponents from its properties.
        hopper.finalizeFromProperties();

        // Show all Components in the model.
        showSubcomponentInfo(hopper);
        // Show only the Joints in the model.
        showSubcomponentInfo<Joint>(hopper);
        // Show the outputs generated by the thigh body.
        showAllOutputs(hopper.getComponent("/Dennis/thigh"), false);

        // Step 1, Task A
        // ==============
        // Determine the name of the output corresponding to the hopper's
        // height. The hopperHeightOutput string (at the top of this file) must
        // be filled in.

        // Step 1, Task B
        // ==============
        // Report the hopper's height and vastus muscle activation during the
        // simulation. The addConsoleReporterToHopper() method (in this file)
        // needs to be filled in.
        addConsoleReporterToHopper(hopper);

        // Create the system, initialize the state, and simulate.
        SimTK::State& sHop = hopper.initSystem();
        simulate(hopper, sHop);
    }

    //==========================================================================
    // Step 2. Build an assistive device and test it on a simple testbed.
    //==========================================================================
    if (true)
    {
        // Build the testbed and device.
        auto testbed = buildTestbed();
        testbed.finalizeFromProperties();

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
        device->finalizeFromProperties();

        // Show all Components in the device and testbed.
        showSubcomponentInfo(*device);
        showSubcomponentInfo(testbed);
        // Show the outputs generated by the device.
        showAllOutputs(*device, false);

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
        std::vector<std::string> deviceOutputs{ "length", "tension", "power",
                                                "controller/myo_control" };

        // Add a ConsoleReporter to report deviceOutputs.
        addDeviceConsoleReporterToModel(testbed, *device, deviceOutputs);

        // Create the system, initialize the state, and simulate.
        SimTK::State& sDev = testbed.initSystem();
        simulate(testbed, sDev);
    }

    //==========================================================================
    // Step 3. Connect the device to the hopper to increase hop height.
    //==========================================================================
    if (true)
    {
        // Build the hopper and device.
        auto assistedHopper = buildHopper();
        assistedHopper.finalizeFromProperties();
        auto kneeDevice = buildDevice();
        kneeDevice->finalizeFromProperties();

        // Step 3, Task A
        // ==============
        // Connect the device to the hopper. The thighAttachment and
        // shankAttachment strings (at the top of this file) must be filled in.
        connectDeviceToModel(*kneeDevice, assistedHopper, thighAttachment,
                             shankAttachment);

        // Step 3, Task B
        // ==============
        // Use the vastus muscle's activation as the control signal for the
        // device. The signalForKneeDevice string (at the top of this file) must
        // be filled in.
        kneeDevice->updInput("controller/activation")
            .connect(assistedHopper.getOutput(vastusActivationOutput));

        // List the device outputs we wish to display during the simulation.
        std::vector<std::string> kneeDeviceOutputs{ "controller/myo_control",
                                                    "tension", "height" };

        // Add a ConsoleReporter to report deviceOutputs.
        addDeviceConsoleReporterToModel(assistedHopper, *kneeDevice,
                                        kneeDeviceOutputs);

        // Create the system, initialize the state, and simulate.
        SimTK::State& sHD = assistedHopper.initSystem();
        simulate(assistedHopper, sHD);
    }

    return 0;
};
