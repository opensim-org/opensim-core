/* ------------------------------------------------------------------------- *
*                OpenSim:  ExampleHopperAssistDevice.cpp                     *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2016 Stanford University and the Authors                *
* Author(s): Chris Dembia, Shrinidhi K. Lakshmikanth, Ajay Seth              *
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

/* This example was designed to present and demonstrate some of the key new
features of the OpenSim 4.0 API. The Component paradigm is more complete,
rigorous and functional to enable models of devices (e.g. sub-assemblies) to
be embedded in other models, for example to a lower extremity or whole body.
Components handle their dependencies consistently and with better error
messaging using Connectors and information flow is enabled by Inputs and
Outputs. Components are easier to construct and generate Outputs, which can
be reported using new Data Components. We will exercise these features in 
this interactive example. */

#include <OpenSim/OpenSim.h>

// Some model and device values that are useful to have
static const double OPTIMAL_FORCE{ 4000 };
static const double GAIN{ 1.0 };
static const double LOAD{ 2500.0 };
static const double SPRINGSTIFF{ 5000 };
static const double SIGNALGEN{ 0.33 };
static const double REPORTING_INTERVAL{ 0.20 };

// Configure which hopper model to use and the attachments (by frame name)
// and any signals (Outputs) for the device that will be created.
static const std::string HopperModelFile{ "BouncingLeg.osim" };
static const std::string DeviceAttachmentA{ "/bouncing_leg/thigh_attachment2" };
static const std::string DeviceAttachmentB{ "/bouncing_leg/shank_attachment2" };
static const std::string SignalForKneeDevice{
                            "/bouncing_leg/vastus/activation" };
static const std::string HopperHeightOutput{
                            "/bouncing_leg/ground_block/yTranslation/value" };


namespace OpenSim {

/* We begin by creating a class to conatin all the parts for the model of
our device. This is similar to how OpenSim uses Model to hold the parts
of a musculoskeletal model. In a Model the parts are ModelComponents.
Since the device will eventually be mounted on a musculoskeletal Model,
we'll make it a type of ModelComponent. Since Components can be composed
by subcomponents by their nature, we don't need to implement any additional
functionality. Later we will add methods to evaluate the performance of the
device. This will be the scaffold for our device, so we'll call the class,
Device. */
class Device : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(Device, Component);

public:
    /** Add outputs so we can report device quantities we care about. */
    /** The length of the device from anchor to anchor point. */
    OpenSim_DECLARE_OUTPUT(length, double, getLength, SimTK::Stage::Position);
    /** The lengthening speed of the device. */
    OpenSim_DECLARE_OUTPUT(speed, double, getSpeed, SimTK::Stage::Velocity);
    /** The force transmitted by the device. */
    OpenSim_DECLARE_OUTPUT(tension, double, getTension, SimTK::Stage::Dynamics);
    /** The power produced(+)/dissipated(-) by the device. */
    OpenSim_DECLARE_OUTPUT(power, double, getPower, SimTK::Stage::Dynamics);
    /** The height of the model that device is attached to. */
    OpenSim_DECLARE_OUTPUT(height, double, getHeight, SimTK::Stage::Position);
    /** The COM height of the model that device is attached to. */
    OpenSim_DECLARE_OUTPUT(com_height, double, getCenterOfMassHeight,
        SimTK::Stage::Position);

    /** Member functions to access values of interest from the device. */
    double getLength(const SimTK::State& s) const {
        return getComponent<PathActuator>("cableAtoB").getLength(s);
    }
    double getSpeed(const SimTK::State& s) const {
        return getComponent<PathActuator>("cableAtoB").getLengtheningSpeed(s);
    }
    double getTension(const SimTK::State& s) const {
        return getComponent<PathActuator>("cableAtoB").computeActuation(s);
    }
    double getPower(const SimTK::State& s) const {
        return getComponent<PathActuator>("cableAtoB").getPower(s);
    }

    //device can read model height as measured from the block to ground
    double getHeight(const SimTK::State& s) const {
        return getModel().getOutputValue<double>(s, HopperHeightOutput);
    }

    //device can also "sense" the model center of mass height 
    double getCenterOfMassHeight(const SimTK::State& s) const {
        SimTK::Vec3 compos = getModel().calcMassCenterPosition(s);
        return compos[1];
    }


protected:
    /** Optionally change the color of the device's actuator path
        as its tension changes. */
    void extendRealizeDynamics(const SimTK::State& s) const override {
        const auto& actuator = getComponent<PathActuator>("cableAtoB");
        double level = fmin(1.0, getTension(s) / actuator.get_optimal_force());
        actuator.getGeometryPath().setColor(s, SimTK::Vec3(level, 0.9, 0.1));
    }
}; // end Device

/**
* Create a Controller that produces a control signal = k * a, where `k` is
* the gain property, and `a` is the activation input. This is intended to model
* proportional myoelectric device controllers. This Controller can control any
* ScalarActuator. The ScalarActuator that this control controls is set using
* the `device` connector.
* http://en.wikipedia.org/wiki/Proportional_Myoelectric_Control
*/
class PropMyoController : public OpenSim::Controller {
    OpenSim_DECLARE_CONCRETE_OBJECT(PropMyoController, OpenSim::Controller);
public:
    OpenSim_DECLARE_PROPERTY(gain, double,
        "Gain used in converting muscle activation into a"
        " control signal (units depend on the device)");

    OpenSim_DECLARE_OUTPUT( myo_control, double, computeControl, 
                            SimTK::Stage::Time);

    OpenSim_DECLARE_INPUT(activation, double, SimTK::Stage::Model,
            "The activation signal that this controller's signal is "
            "proportional to.");

    PropMyoController() {
        constructInfrastructure();
    }

    double computeControl(const SimTK::State& s) const {
        double activation = getInputValue<double>(s, "activation");
        if (activation < 0.31)
            return 0;
        // Compute the control signal.
        return get_gain() * activation;
    }

    void computeControls(const SimTK::State& s,
        SimTK::Vector& controls) const override {
        double signal = computeControl(s);
        // Add in this control signal to controls.
        const auto& actuator = getConnectee<Actuator>("actuator");
        SimTK::Vector thisActuatorsControls(1, signal);
        actuator.addInControls(thisActuatorsControls, controls);
    }

private:
    void constructProperties() override {
        constructProperty_gain(1.0);
    }

    void constructConnectors() override {
        // The ScalarActuator for which we're computing a control signal.
        constructConnector<Actuator>("actuator");
    }
}; // end of PropMyoController

/* A Generator is a component with no Inputs and only Outputs. This
SignalGenerator evaluates an OpenSim::Function, specified as a property,
as a function of time determined from the state. It's function is only
evaluated when the Output must provide its value (e.g. to an Input) */
class SignalGenerator : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(SignalGenerator, Component);
public:
    OpenSim_DECLARE_PROPERTY(function, OpenSim::Function,
        "Function used to generate the signal (waveform) w.r.t time.");

    OpenSim_DECLARE_OUTPUT(signal, double, getSignal, SimTK::Stage::Time);

    SignalGenerator() {
        constructInfrastructure();
    }

    double getSignal(const SimTK::State& s) const {
        return get_function().calcValue(SimTK::Vector(1, s.getTime()));
    }
private:
    void constructProperties() override {
        constructProperty_function(Constant(0.0));
    }
}; // end of SignalGenerator



/*************** HELPER FUNCTIONS: Implemented in answers.cpp ****************/
 // Utility to load and draw a model in a refresh loop
 // so that edits to the modelFile can be visualized immediately.
void refreshModel(const std::string& modelFile);

// Helper method to edit a device's path (if it has one) to handle
// wrapping over a wrap surface already in the model.
void handlePathWrapping(OpenSim::ModelComponent* device,
    OpenSim::Model& model);

// Main driver to simulate a model from an initial state
// Simulate means to integrate the model equations forward in time.
// The State is updated so that it returns the final state of integration.
void simulate(OpenSim::Model& model, SimTK::State& state);
/************ end of HELPER FUNCTIONS: Implemented in answers.cpp ************/


OpenSim::Model createTestBed() {
    using SimTK::Vec3;
    using SimTK::Inertia;

    OpenSim::Model testBed;
    testBed.setName("testbed");
    testBed.setUseVisualizer(true);
    testBed.setGravity(Vec3(0));

    // Create a load of mass LOAD kg.
    auto load = new OpenSim::Body("load", LOAD, Vec3(0), Inertia(1));
    // Set properties of a sphere geometry to be used for the load.
    OpenSim::Sphere sphere;
    sphere.setFrameName("load");
    sphere.set_radius(0.02);
    sphere.setOpacity(0.5);
    sphere.setColor(Vec3{ 0, 0, 1 });
    load->addGeometry(sphere);
    testBed.addBody(load);

    auto grndToLoad = new OpenSim::FreeJoint("grndToLoad", "ground", "load");
    // Set the location of the load to (1, 0, 0).
    grndToLoad->getCoordinateSet()[3].setDefaultValue(1.0);
    testBed.addJoint(grndToLoad);

    auto spring = new OpenSim::PointToPointSpring(
        testBed.getGround(), Vec3(0),// point 1's frame and location in that frame
        *load, Vec3(0),              // point 2's frame and location in that frame
        SPRINGSTIFF, 1.0);           // spring stiffness and rest-length

    testBed.addForce(spring);

    return testBed;
}

// Use any two (PhysicalFrame) frame's in a model to attach the device
void connectDeviceToModel(const std::string& frameAname,
                          const std::string& frameBname,
                          OpenSim::Device* device, OpenSim::Model& model) {

    //Get the known anchors (joints) that attach the device to a model
    auto& anchorA = device->updComponent<OpenSim::Joint>("anchorA");
    auto& anchorB = device->updComponent<OpenSim::Joint>("anchorB");
    // Attach anchorA to frameA as anchor's (joint's) parent frame.
    anchorA.setParentFrameName(frameAname);
    // Attach anchorB to frameB as anchor's (joint's) parent frame.
    anchorB.setParentFrameName(frameBname);

    // handle wrapping if there is a wrap surface between the device
    // origin and insertion on the model.
    handlePathWrapping(device, model);

    // Add the device to the testBed.
    model.addModelComponent(device);
}

} // namespace OpenSim


// Build the Device composed of 1kg "cuffs" that attach to two frames in a
// model. Between these two cuffs there is an actuator that can conform 
// to wrap over a joint. The actuator receives its control from its controller.
// The controller generates a control signal proportional to an "activation"
// signal, which is an output of an unknown model.
OpenSim::Device* createDevice() {
    using SimTK::Vec3;
    using SimTK::Inertia;

    //-----------------Code to Assemble the Device begin -----------------------
    // Create the device to hold the components.
    auto device = new OpenSim::Device{};
    device->setName("device");

    // Mass of the device distributed between two cuffs that attach to a
    // model (person, test-bed). Each cuff has a mass of 1 kg, center of mass
    // at the origin of their respective frames, and moment of inertia of 0.5
    // and products of zero.
    auto cuffA = new OpenSim::Body("cuffA", 1, Vec3(0), Inertia(0.5));
    auto cuffB = new OpenSim::Body("cuffB", 1, Vec3(0), Inertia(0.5));
    // Add the masses to the device.
    device->addComponent(cuffA);
    device->addComponent(cuffB);

    // Create a sphere geometry to visually represent the cuffs
    OpenSim::Sphere sphere{ 0.01 };
    sphere.setName("sphere");
    sphere.setColor(Vec3{ 0.0, 0.8, 0 });
    // Add sphere (geometry) attach them to the cuffs
    sphere.setFrameName("cuffA");
    cuffA->addGeometry(sphere);
    sphere.setFrameName("cuffB");
    cuffB->addGeometry(sphere);

    // Joint from something in the environment to cuffA.
    // It will be used to attach the device at cuffA to a model.
    auto anchorA = new OpenSim::WeldJoint();
    anchorA->setName("anchorA");
    // Set only the child now. Parent will be in the environment.
    anchorA->setChildFrameName("cuffA");
    device->addComponent(anchorA);

    // Joint from something in the environment to cuffB.
    // It will be used to attach the device at cuffA to a model.
    auto anchorB = new OpenSim::WeldJoint();
    anchorB->setName("anchorB");
    // Set only the child now. Parent will be in the environment.
    anchorB->setChildFrameName("cuffB");
    device->addComponent(anchorB);

    // Actuator connecting the two cuffs (A and B).
    auto pathActuator = new OpenSim::PathActuator();
    pathActuator->setName("cableAtoB");
    pathActuator->set_optimal_force(OPTIMAL_FORCE);
    pathActuator->addNewPathPoint("point1", *cuffA, Vec3(0));
    pathActuator->addNewPathPoint("point2", *cuffB, Vec3(0));
    device->addComponent(pathActuator);

    // A controller that specifies the control to the actuator
    auto controller = new OpenSim::PropMyoController();
    controller->setName("controller");
    controller->set_gain(GAIN);

    // Connect the the controller to the device actuator
    controller->updConnector<OpenSim::Actuator>("actuator").
        connect(*pathActuator);

    // Don't forget to add the controller to your device
    device->addComponent(controller);

    return device;
}

/* Create and add a Reporter to a model that reports device outputs 
   as listed by name. */
void addDeviceReporterToModel(OpenSim::Device& device, OpenSim::Model& model,
                        const std::vector<std::string>& deviceOutputs)
{
    auto reporter = new OpenSim::ConsoleReporter();
    reporter->setName(model.getName() +"_"+ device.getName()+"_results");
    reporter->set_report_time_interval(REPORTING_INTERVAL);

    // loop through desired device outputs by name
    for (auto outputName : deviceOutputs) {
        reporter->updInput("inputs").connect(device.getOutput(outputName));
        //std::cout << "Connected Output: " << outputName << std::endl;
    }
    model.addComponent(reporter);
    //std::cout << "Added reporter '" << reporter->getName()  << 
    //    "' to model '" << model.getName() << "'." << std::endl;
}

void addReporterToHopper(OpenSim::Model& hopper) {
    auto reporter = new OpenSim::ConsoleReporter();
    reporter->setName("hopper_results");
    reporter->set_report_time_interval(REPORTING_INTERVAL);
    // A reporter has a multi channel Input called inputs that can connect to
    // any number of Outputs as long as they are of type double.
    reporter->updInput("inputs")
        .connect(hopper.getOutput(SignalForKneeDevice));
    reporter->updInput("inputs")
        .connect(hopper.getOutput(HopperHeightOutput));
    // don't forget to add the reporter to the hopper
    hopper.addComponent(reporter);
}

void addSignalGeneratorToDevice(OpenSim::Device* device) {
    auto generator = new OpenSim::SignalGenerator();
    generator->setName("generator");
    // Trying changing the constant value and even changing
    // the function, e.g. try a LinearFunction
    generator->set_function(OpenSim::Constant(SIGNALGEN));
    device->addComponent(generator);
    // Wire up the Controller to use the generator for fake activations
    device->updInput("controller/activation").
        connect(generator->getOutput("signal"));
}


int main() {
    using namespace OpenSim;
    //----------------------------- HOPPER CODE begin --------------------------
    // Load the hopper model and simulate (unassisted)
    Model hopper(HopperModelFile);
    hopper.setUseVisualizer(true);

    ForceReporter* fp = new ForceReporter(&hopper);
    fp->setOn(true);
    fp->setPrintResultFiles(true);

    hopper.addAnalysis(fp);

    /**** EXERCISE 1: Add a Console Reporter ***********************************
    /* Report the models height and muscle activation during the simulation.   *
    /***************************************************************************/
    addReporterToHopper(hopper);
    /**** EXERCISE 1: end *****************************************************/
    
    // Create the system and initialize the corresponding state an return it
    SimTK::State& sH = hopper.initSystem();
    // Simulate the hopper from the initial state. The state is updated during
    // the simulation.
    simulate(hopper, sH);
    fp->printResults("hopperForces");
    //----------------------------- HOPPER CODE end ----------------------------

    //--------------------------- DEVICE CODE begin ----------------------------
    /**** EXERCISE 2: Create the Device ****************************************
    /* Populate a Device instance with parts you need: anchors, actuator,...   *
    /***************************************************************************/
    auto device = createDevice();
    /**** EXERCISE 2: end *****************************************************/

    // Create a test environment for the device. You can comment out the call
    // when connecting the device built above to the actual hopper because this
    // testBed is actually for testing this device.
    //-------------------------------------------------------------------------
    auto testBed = createTestBed(); // or load a model

    // Connect device to a parent model. 
    // Here we connect the device to the testBed at its ground and load frames
    //-------------------------------------------------------------------------
    connectDeviceToModel("ground", "load", device, testBed);

    /**** EXERCISE 3: Create a Signal Generator ********************************
    /* Make a SignalGenerator class and use it to input signals to test device *
    /***************************************************************************/
    addSignalGeneratorToDevice(device);
    /**** EXERCISE 3: end *****************************************************/

    // list desired device outputs (values of interest) by name
    std::vector<std::string> deviceOutputs{ "length", "tension",
                                "power", "controller/myo_control" };
    // add a ConsoleReporter to report device values during a simulation
    addDeviceReporterToModel(*device, testBed, deviceOutputs);

    auto& sD = testBed.initSystem();

    // Simulate the testBed containing the device only.
    simulate(testBed, sD);
    //----------------------------- DEVICE CODE end ---------------------------

    //---------------------------- HOPPER + DEVICE begin ----------------------
    /**** EXERCISE 4: Simulate Hopper with the Device **************************
    /* Make a SignalGenerator class and use it to input signals to test device *
    /***************************************************************************/
    // Begin by loading the hopper from file and then we'll connect the device.
    Model hopperWithDevice(HopperModelFile);
    hopperWithDevice.setUseVisualizer(true);

    // Make a copy (clone) of the device as a knee specific device to connect
    // to the hopper model
    Device* kneeDevice = device->clone();
    kneeDevice->finalizeFromProperties();

    // Connect the kneeDevice to the hopper so it really becomes hopperWithDevice
    connectDeviceToModel(DeviceAttachmentA, DeviceAttachmentB, 
                         kneeDevice, hopperWithDevice);

    // Hook-up the device's controller input ("activation") to its signal, which
    // is an Output from the hopper corresponding to the vastus muscle activation
    kneeDevice->updInput("controller/activation")
        .connect(hopperWithDevice.getOutput(SignalForKneeDevice));

    // List desired device outputs (values of interest) by name
    std::vector<std::string> deviceOutputs2{ "controller/myo_control",
                                             "tension", "height" };
    // add a ConsoleReporter to report device values during a simulation
    addDeviceReporterToModel(*kneeDevice, hopperWithDevice, deviceOutputs2);

    // Simulate the hopper with the device.
    SimTK::State& sHD = hopperWithDevice.initSystem();
    simulate(hopperWithDevice, sHD);
    /**** EXERCISE 4: end *****************************************************/
    //----------------------------- HOPPER + DEVICE end ------------------------

    return 0;
};


