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
rigorous and functional to enable models of devices or sub-assemblies to be
embedded in other models, for example of the lower extremity or whole body.
Components handle their dependencies consistently and with better error
messaging using Connectors and information flow is enabled by Inputs and
Outputs. Components are easier to construct and generate Outputs, which can
be reported using new Data Components. */

#define LUXO 0
// or: #define LUXO 1

#if LUXO
    #define OPTIMAL_FORCE 200
    #define MASS 0.1
    #define GAIN 2
    #define LOAD 10
    #define SPRINGSTIFF 5
    #define SIGNALGEN 1
#else
    #define OPTIMAL_FORCE 80
    #define GAIN 10
    #define LOAD 1000
    #define SPRINGSTIFF 500
    #define SIGNALGEN 0.1
#endif

#include <OpenSim/OpenSim.h>

namespace OpenSim {

    /* We begin by creating a container to hold all the parts for the model of
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
    protected:
        virtual void extendRealizeDynamics(const SimTK::State& s) const override {
            const auto& actu = getComponent<PathActuator>("cableAtoB");
            actu.getGeometryPath().setColor(s,
                    SimTK::Vec3(fmin(1.0,
                            getTension(s) / actu.get_optimal_force())));
        }

    };

    /**
    * Create a Controller that produces a control signal = k * a, where `k` is
    * the gain property, and `a` is the activation input. This is intended to model
    * proportional myoelectric device controllers. This Controller can control any
    * ScalarActuator. The ScalarActuator that this control controls is set using
    * the `device` connector.
    *
    * http://en.wikipedia.org/wiki/Proportional_Myoelectric_Control
    */
    class PropMyoController : public OpenSim::Controller {
        OpenSim_DECLARE_CONCRETE_OBJECT(PropMyoController, OpenSim::Controller);
    public:
        OpenSim_DECLARE_PROPERTY(gain, double,
            "Gain used in converting muscle activation into a"
            " control signal (units depend on the device)");
        OpenSim_DECLARE_OUTPUT(myo_control, double, computeControl, SimTK::Stage::Time);

        OpenSim_DECLARE_INPUT(activation, double, SimTK::Stage::Model,
                "The activation signal that this controller's signal is "
                "proportional to.");

        PropMyoController() {
            constructInfrastructure();
        }

        double computeControl(const SimTK::State& s) const {
            double activation = getInputValue<double>(s, "activation");
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
    };

    /* A simple Component with no Inputs and only one Output. It evaluates
    an OpenSim::Function, specified as a property, as a function of time
    determined from the state. It's function is only evaluated when the
    Output must provide its value (e.g. to an Input) */
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
    };
} // namespace OpenSim


void getAnswer();

OpenSim::Model createTestBed() {
    using SimTK::Vec3;
    using SimTK::Inertia;

    OpenSim::Model testBed;
    testBed.setUseVisualizer(true);
    testBed.setGravity(Vec3(0));

    // Create a load of mass 1kg.
    auto load = new OpenSim::Body("load", 1, Vec3(0), Inertia(1));
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
        testBed.getGround(), Vec3(0), // point 1's frame and location in that frame
        *load, Vec3(0),               // point 2's frame and location in that frame
        SPRINGSTIFF, 1.0);                   // spring stiffness and rest-length

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
    // Add the device to the testBed.
    model.addModelComponent(device);
}

// Simulate any model from an initial state
void simulate(OpenSim::Model& model, SimTK::State& state) {
    SimTK::State s0 = state;
    // Configure the 3D visualizer environment
    if (model.getUseVisualizer()) {
        model.updMatterSubsystem().setShowDefaultGeometry(false);
        SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundType(viz.GroundAndSky);
        viz.setShowSimTime(true);
        viz.drawFrameNow(state);
        // wait for user input to start the simulation.
        std::cout << "Press enter/return to continue:" << std::endl;
        std::cin.get();
    }

    // Simulate in a replay loop if necessary
    char replay = 'r';
    while (replay == 'r') {
        state = s0;
        SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
        OpenSim::Manager manager(model, integrator);
        manager.setInitialTime(0);
        manager.setFinalTime(5.0);
        manager.integrate(state);
        // generate states output debugging
        manager.getStateStorage().print("exampleHopperStates.sto");

        // wait for user input to proceed.
        std::cout << "Press enter/return to continue OR 'r' to replay." << std::endl;
        std::cin >> replay;
    }
}

OpenSim::Device* createDevice() {
    using SimTK::Vec3;
    using SimTK::Inertia;

    //-----------------Code to Assemble the Device begin -----------------------
    // Create a sphere geometry to reuse later.
    OpenSim::Sphere sphere{ 0.01 };
    sphere.setName("sphere");
    sphere.setColor(Vec3{ 0, 1, 0 });

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

    // Sphere geometry for the masses. 
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

    // Actuator connecting the two masses.
    auto pathActuator = new OpenSim::PathActuator();
    pathActuator->setName("cableAtoB");
    pathActuator->set_optimal_force(OPTIMAL_FORCE);
    pathActuator->addNewPathPoint("point1", *cuffA, Vec3(0));
    pathActuator->addNewPathPoint("point2", *cuffB, Vec3(0));
    device->addComponent(pathActuator);

    // A controller that specifies the excitation of the biceps muscle.
    auto controller = new OpenSim::PropMyoController();
    controller->setName("controller");
    controller->set_gain(GAIN);

    controller->updConnector<OpenSim::Actuator>("actuator").
        connect(*pathActuator);
    device->addComponent(controller);

    return device;
}

int main() {
    //--------------------------- DEVICE CODE begin ------------------_--------
    auto device = createDevice();
    // Build a test environment for the device. You can comment out the call
    // when connecting the device built above to the actual hopper because this
    // testBed is actually for testing this device.
    //-------------------------------------------------------------------------
    auto testBed = createTestBed(); // or load a model

    // Connect device to a parent model. 
    // Here we connect the device to the testBed at its ground and load frames
    //-------------------------------------------------------------------------
    connectDeviceToModel("ground", "load", device, testBed);
    // Print the model. 
    testBed.print("exampleHopperDeviceOnTestBed.osim");

    auto generator = new OpenSim::SignalGenerator();
    generator->setName("generator");
    // Trying changing the constant value and even changing
    // the function, e.g. try a LinearFunction
    generator->set_function(OpenSim::Constant(SIGNALGEN));
    device->addComponent(generator);
    // Wire up the Controller to use the generator for fake activations
    device->updInput("controller/activation").
        connect(generator->getOutput("signal"));

    auto reporter = new OpenSim::ConsoleReporter();
    reporter->setName("results");
    reporter->set_report_time_interval(0.5);
    reporter->updInput("inputs").connect(device->getOutput("length"));
    reporter->updInput("inputs").connect(device->getOutput("tension"));
    reporter->updInput("inputs").connect(device->getOutput("power"));
    reporter->updInput("inputs").connect(device->getOutput("controller/myo_control"));

    testBed.addComponent(reporter);

    auto& state = testBed.initSystem();

    // Simulate the testBed containing the device only. When using the hopper,
    // make sure to simulate the hopper (with the device) and not the testBed.
    //simulate(testBed, state);
    //----------------------------- DEVICE CODE end --------------------------


    //----------------------------- HOPPER CODE begin --------------------------
    std::string modelFile, attachmentA, attachmentB, signalForDevice;
    if (LUXO) {
        modelFile = "Luxo_Myo.osim";
        //attachmentA = "knee_assist_origin";
        //attachmentB = "knee_assist_insertion"; 
        attachmentA = "back_assist_origin";
        attachmentB = "back_assist_insertion";
        signalForDevice = "/LuxoMuscle/back_extensor_right/activation";
    } else {
        modelFile = "bouncing_block.osim";
        attachmentA = "/toy_with_forces/thigh_attachment";
        attachmentB = "/toy_with_forces/shank_attachment";
        signalForDevice = "/toy_with_forces/vastus/activation";
    }
    OpenSim::Model luxo(modelFile);
    luxo.setUseVisualizer(true);


    auto reporterH = new OpenSim::ConsoleReporter();
    reporterH->setName("resultsH");
    reporterH->set_report_time_interval(0.5);
    reporterH->updInput("inputs").connect(luxo.getOutput(signalForDevice));
    luxo.addComponent(reporterH);

    // TODO put this back in SimTK::State& sH = luxo.initSystem();
    // TODO put this back in simulate(luxo, sH);
    //----------------------------- HOPPER CODE end ----------------------------


    //----------------------------- HOPPER + DEVICE begin ----------------------
    OpenSim::Device* luxoDevice = device->clone();
    luxoDevice->finalizeFromProperties();
    connectDeviceToModel(attachmentA, attachmentB, luxoDevice, luxo);

    // set the controller 
    luxoDevice->updInput("controller/activation").connect(luxo.getOutput(signalForDevice));


    // Add some more quantities to report.
    reporterH->updInput("inputs").connect(luxoDevice->getOutput("controller/myo_control"));
    reporterH->updInput("inputs").connect(luxoDevice->getOutput("tension"));
    //----------------------------- HOPPER + DEVICE end ------------------------

    SimTK::State& sHD = luxo.initSystem();
    // Simulate the Luxo model with the device. 
    simulate(luxo, sHD);
    //------------------------------ ANALYZE begin -----------------------------
    // TO DO -- Your analysis code goes here.
    //------------------------------ ANALYZE end -------------------------------

    getAnswer();
};
