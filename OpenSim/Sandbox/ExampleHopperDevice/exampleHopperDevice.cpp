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
static const double REPORTING_INTERVAL{ 0.2 };

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

/* We begin by creating a class to contain all the parts for the model of
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

    // TODO: add other outputs.

    /** Member functions to access values of interest from the device. */
    double getLength(const SimTK::State& s) const {
        return getComponent<PathActuator>("cableAtoB").getLength(s);
    }

    // TODO: add other output functions.

protected:
    /** Optionally change the color of the device's actuator path
        as its tension changes. */
    void extendRealizeDynamics(const SimTK::State& s) const override {
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

    // TODO: gain property

    // TODO: myo_control output

    OpenSim_DECLARE_INPUT(activation, double, SimTK::Stage::Model,
            "The activation signal that this controller's signal is "
            "proportional to.");

    PropMyoController() {
        constructInfrastructure();
    }

    double computeControl(const SimTK::State& s) const {
        // Compute the proportional control of GAIN * activation (Input)
        // TODO
        return 0;
    }

    void computeControls(const SimTK::State& s,
        SimTK::Vector& controls) const override {
        double signal = computeControl(s);
        // Add in this control signal to controls.
        // TODO
        // const auto& actuator = getConnectee<Actuator>("actuator");
        SimTK::Vector thisActuatorsControls(1, signal);
        // Add in this controller's controls for the actuator
        // TODO
    }

private:

    // TODO constructProperties()

    void constructConnectors() override {
        // The ScalarActuator for which we're computing a control signal.
        // TODO
        
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
    // TODO

    // Attach anchorA to frameA as anchor's (joint's) parent frame.
    // TODO

    // Attach anchorB to frameB as anchor's (joint's) parent frame.
    // TODO


    // handle wrapping if there is a wrap surface between the device
    // origin and insertion on the model.
    // TODO

    // Add the device to the testBed.
    // TODO

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
    // TODO: cuffB

    // Add the cuffs to the device.
    // TODO


    // Create a sphere geometry to visually represent the cuffs
    OpenSim::Sphere sphere{ 0.01 };
    sphere.setName("sphere");
    sphere.setColor(Vec3{ 0.0, 0.8, 0 });
    // Add sphere (geometry) attach them to the cuffs
    sphere.setFrameName("cuffA");
    cuffA->addGeometry(sphere);
    // TODO: cuffB

    // Joint from something in the environment to cuffA.
    // It will be used to attach the device at cuffA to a model.
    auto anchorA = new OpenSim::WeldJoint();
    // TODO: set name

    // Set only the child now. Parent will be in the environment.
    // TODO


    // Joint from something in the environment to cuffB.
    // It will be used to attach the device at cuffA to a model.
    // TODO

    // PathActuator connecting the two cuffs (A and B).
    // TODO

    // A controller that specifies the control to the actuator
    // TODO
    // TODO: finish implementing the PropMyoController class, above.

    // Connect the controller to the device actuator
    // TODO

    // Don't forget to add the controller to your device
    // TODO

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
    // TODO
}

void addSignalGeneratorToDevice(OpenSim::Device* device) {
    // TODO
}


int main() {
    using namespace OpenSim;
    //----------------------------- HOPPER CODE begin --------------------------
    // Load the hopper model and simulate (unassisted)
    Model hopper(HopperModelFile);
    hopper.setUseVisualizer(true);

    /**** EXERCISE 1: Add a Console Reporter ***********************************
     * Report the models height and muscle activation during the simulation.   *
     ***************************************************************************/
    addReporterToHopper(hopper);
    /**** EXERCISE 1: end *****************************************************/
    
    // Create the system and initialize the corresponding state an return it
    SimTK::State& sH = hopper.initSystem();
    // Simulate the hopper from the initial state. The state is updated during
    // the simulation.
    simulate(hopper, sH);
    //----------------------------- HOPPER CODE end ----------------------------

    //--------------------------- DEVICE CODE begin ----------------------------
    /**** EXERCISE 2: Create the Device ****************************************
     * Populate a Device instance with parts you need: anchors, actuator,...   *
     ***************************************************************************/
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
     * Make a SignalGenerator class and use it to input signals to test device *
     ***************************************************************************/
    addSignalGeneratorToDevice(device);
    /**** EXERCISE 3: end *****************************************************/

    // list desired device outputs (values of interest) by name
    std::vector<std::string> deviceOutputs{ "length", "tension",
                                "power", "controller/myo_control" };
    // add a ConsoleReporter to report device values during a simulation
    // addDeviceReporterToModel(*device, testBed, deviceOutputs);

    // initialize the system and the initial state
    // auto& sD = testBed.initSystem();

    // Simulate the testBed containing the device only.
    // TODO

    //----------------------------- DEVICE CODE end ---------------------------

    //---------------------------- HOPPER + DEVICE begin ----------------------
    /**** EXERCISE 4: Simulate Hopper with the Device **************************
     * Combine Hopper and Device models to simulate an assisted jump           *
     ***************************************************************************/
    // Begin by loading the hopper from file and then we'll connect the device.
    // TODO

    // Make a copy (clone) of the device as a knee specific device to connect
    // to the hopper model
    // TODO

    // Connect the kneeDevice to the hopper so it really becomes hopperWithDevice
    // TODO

    // Hook-up the device's controller input ("activation") to its signal, which
    // is an Output from the hopper corresponding to the vastus muscle activation
    // TODO

    // List desired device outputs (values of interest) by name
    // TODO

    // add a ConsoleReporter to report device values during a simulation
    // TODO

    // Simulate the hopper with the device.
    // TODO

    /**** EXERCISE 4: end *****************************************************/
    //----------------------------- HOPPER + DEVICE end ------------------------

    return 0;
};


