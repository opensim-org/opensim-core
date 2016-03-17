#include <OpenSim/OpenSim.h>

namespace OpenSim {

class Composite : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(Composite, Component);

public:
    OpenSim_DECLARE_LIST_PROPERTY(components, Component, 
                                  "List of serialized internal components");

    Composite() {
        constructInfrastructure();
    }

    void adopt(Component* comp) {
        // add it the property list of components that owns and serializes them
        updProperty_components().adoptAndAppendValue(comp);
        finalizeFromProperties();
    }

private:
    void constructProperties() override {
        constructProperty_components();
    }
};

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
        constructProperty_function(Constant(0.9876543));
    }
};


/**
 * This produces a control signal k * a, where `k` is the gain property, and
 * `a` is the activation input. This is intended to model proportional
 * myoelectric device controllers. This Controller can control any
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
        const auto& device = getConnectee<Actuator>("device");
        SimTK::Vector thisActuatorsControls(1, signal);
        device.addInControls(thisActuatorsControls, controls);
    }

private:

    void constructProperties() override {
        constructProperty_gain(1.0);
    }

    void constructConnectors() override {
        // The ScalarActuator for which we're computing a control signal.
        constructConnector<Actuator>("device");
    }

    void constructInputs() override {
        // The control signal is proportional to this input.
        constructInput<double>("activation", SimTK::Stage::Model);
    }


};

} // namespace OpenSim


OpenSim::Model createTestBed(OpenSim::Sphere& sphere) {
    using SimTK::Vec3;
    using SimTK::Inertia;

    OpenSim::Model testBed; 
    testBed.setUseVisualizer(true);
    testBed.setGravity(Vec3(0));

    // Create a load of mass 10kg.
    auto load = new OpenSim::Body("load", 10, Vec3(0), Inertia(1));
    // Set properties of the sphere geometry to be used for the load.
    sphere.setFrameName("load");
    sphere.set_radius(0.2);
    sphere.setOpacity(0.5);
    sphere.setColor(Vec3{0, 0, 1});
    load->append_geometry(sphere);
    testBed.addBody(load);

    auto grndToLoad = new OpenSim::FreeJoint("grndToLoad", "ground", "load");
    // Set the location of the load to (1, 0, 0).
    grndToLoad->getCoordinateSet()[3].setDefaultValue(1);
    testBed.addJoint(grndToLoad);

    return testBed;
}

void connectDeviceToTestBed(OpenSim::Joint* anchorA,
                            OpenSim::Joint* anchorB,
                            OpenSim::Composite* device,
                            OpenSim::Model& testBed) {
    // Set parent of anchorA as ground.
    anchorA->setParentFrameName("ground");
    // Set parent of anchorB as load.
    anchorB->setParentFrameName("load");
    // Add the device to the testBed.
    testBed.addModelComponent(device);
}

void simulate(OpenSim::Model& model, SimTK::State& state) {
    // Print the model. 
    model.print("exampleHopperDevice.xml");

    // Configure the model.
    //auto& state = model.initSystem();

    model.updMatterSubsystem().setShowDefaultGeometry(true);

    // Simulate.
    SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
    OpenSim::Manager manager(model, integrator);
    manager.setInitialTime(0); 
    manager.setFinalTime(10.0);
    manager.integrate(state);

    manager.getStateStorage().print("exampleHopperStates.sto");
}

int main() {
    using SimTK::Vec3;
    using SimTK::Inertia;
    using SimTK::Pi;

    //----------------------------- HOPPER CODE begin --------------------------
    // TO DO -- Your code related to hopper goes here.
    //----------------------------- HOPPER CODE end ----------------------------


    //----------------------------- DEVICE CODE begin --------------------------

    // Create a sphere geometry to reuse later.
    OpenSim::Sphere sphere{0.1};
    sphere.setName("sphere");

    // Create the device to hold the components.
    //-------------------------------------------------------------------------
    auto device = new OpenSim::Composite{};
    device->setName("device");

    // Two body(s), with mass of 1 kg, center of mass at the
    // origin of their respective frames, and moments/products of inertia of 
    // zero.
    auto massA = new OpenSim::Body("massA", 1, Vec3(0), Inertia(1));
    auto massB = new OpenSim::Body("massB", 1, Vec3(0), Inertia(1));
    // Add the masses to the device.
    device->adopt(massA);
    device->adopt(massB);

    // Sphere geometry for the masses. 
    sphere.setFrameName("massA");
    massA->append_geometry(sphere);
    sphere.setFrameName("massB");
    massB->append_geometry(sphere);

    // Joint from something in the environment to massA. 
    auto anchorA = new OpenSim::WeldJoint();
    anchorA->setName("anchorA");
    // Set only the child now. Parent will be in the environment.
    anchorA->setChildFrameName("massA");
    device->adopt(anchorA);

    // Joint from something in the environment to massB. 
    auto anchorB = new OpenSim::WeldJoint();
    anchorB->setName("anchorB");
    // Set only the child now. Parent will be in the environment.
    anchorB->setChildFrameName("massB");
    device->adopt(anchorB);

    // Actuator connecting the two masses.
    auto pathActuator = new OpenSim::RigidTendonMuscle("muscleAtoB", 10, 0.5, 0.5, 0);
    //pathActuator->setName("cableAtoB");
    pathActuator->addNewPathPoint("point1", *massA, Vec3(0));
    pathActuator->addNewPathPoint("point2", *massB, Vec3(0));
    device->adopt(pathActuator);

    // A controller that specifies the excitation of the biceps muscle.
    auto controller = new OpenSim::PropMyoController();
    controller->setName("controller");

    auto generator = new OpenSim::SignalGenerator();
    generator->setName("generator");
    device->adopt(generator);

    controller->updConnector<OpenSim::Actuator>("device").
                connect(*pathActuator);
    device->adopt(controller);

    // Build a test environment for the device. Comment the below function call
    // when connecting the device built above to the actual hopper because this
    // testBed is actually for testing this device.
    //-------------------------------------------------------------------------
    auto testBed = createTestBed(sphere);

    // Connect device to/from the environment. Comment the below code and 
    // make sure to connect the device to the actual hopper and simulate with
    // hopper connected to the device.
    //-------------------------------------------------------------------------
    connectDeviceToTestBed(anchorA, anchorB, device, testBed);

    auto& state = testBed.initSystem();

    // temporary bug that Inputs have to be wired up after initSystem
    controller->updInput("activation").
        connect(generator->getOutput("signal"));

    // Simulate the testBed containing the device only. When using the hopper,
    // make sure to simulate the hopper (with the device) and not the testBed.
    simulate(testBed, state);

    //----------------------------- DEVICE CODE begin --------------------------

    //----------------------------- HOPPER + DEVICE begin ----------------------
    // TO DO -- Your code related to hopper with the device goes here.
    //----------------------------- HOPPER + DEVICE end ------------------------


    //------------------------------ ANALYZE begin -----------------------------
    // TO DO -- Your analysis code goes here.
    //------------------------------ ANALYZE end -------------------------------
};
