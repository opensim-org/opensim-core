/* -------------------------------------------------------------------------- *
 *                OpenSim:  testComponentInterface.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Ayman Habib                                          *
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
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Common/Component.h>

using namespace OpenSim;
using namespace std;
using namespace SimTK;

class Foo;
class Bar;

class Sub : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(Sub, Component);
public:
    Sub() = default;
    virtual ~Sub() = default;
}; //end class Sub

class TheWorld : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(TheWorld, Component);
public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_LIST_PROPERTY(components, Component, 
        "List of serialized internal components");

    TheWorld() : Component() {
        // Constructing own properties, connectors, inputs or connectors? Must invoke!
        constructInfrastructure();
    }

    TheWorld(const std::string& fileName, bool updFromXMLNode = false)
        : Component(fileName, updFromXMLNode) {
        // have to construct this Component's properties so that deserialization from
        // XML has a place to go.
        constructInfrastructure();
        // Propagate XML file values to properties 
        updateFromXMLDocument();
        // add components listed as properties as sub components.
        finalizeFromProperties();
    }

    void add(Component* comp) {
        // add it the property list of components that owns and serializes them
        updProperty_components().adoptAndAppendValue(comp);
        finalizeFromProperties();
    }

    // Top level connection method for all encompassing Component
    void connect() {
        Super::connect(*this);
    }
    void buildUpSystem(MultibodySystem& system) { addToSystem(system); }

    const SimbodyMatterSubsystem& getMatterSubsystem() const { return *matter; }
    SimbodyMatterSubsystem& updMatterSubsystem() const { return *matter; }

    const GeneralForceSubsystem& getForceSubsystem() const { return *forces; }
    GeneralForceSubsystem& updForceSubsystem() const { return *forces; }

protected:
    // Component interface implementation
    void extendAddToSystem(MultibodySystem& system) const override {
        if (system.hasMatterSubsystem()){
            matter = system.updMatterSubsystem();
        }
        else{
            SimbodyMatterSubsystem* old_matter = matter.release();
            delete old_matter;
            matter = new SimbodyMatterSubsystem(system);

            GeneralForceSubsystem* old_forces = forces.release();
            delete old_forces;
            forces = new GeneralForceSubsystem(system);

            SimTK::Force::UniformGravity gravity(*forces, *matter, Vec3(0, -9.816, 0));
            fix = gravity.getForceIndex();

            system.updMatterSubsystem().setShowDefaultGeometry(true);
        }
    }

private:
    void constructProperties() override {
        constructProperty_components();
    }

private:
    // Keep track of pointers to the underlying computational subsystems. 
    mutable ReferencePtr<SimbodyMatterSubsystem> matter;
    mutable ReferencePtr<GeneralForceSubsystem> forces;

    // keep track of the force added by the component
    mutable ForceIndex fix;

    Sub internalSub{ constructSubcomponent<Sub>("internalSub") };

}; // end of TheWorld


class Foo : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(Foo, Component);
public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(mass, double, "mass (kg)");
    OpenSim_DECLARE_LIST_PROPERTY_SIZE(inertia, double, 6,
        "inertia {Ixx, Iyy, Izz, Ixy, Ixz, Iyz}");

    OpenSim_DECLARE_OUTPUT(Output1, double, getSomething, SimTK::Stage::Time)
    OpenSim_DECLARE_OUTPUT(Output2, SimTK::Vec3, calcSomething,
            SimTK::Stage::Time)

    OpenSim_DECLARE_OUTPUT(Qs, Vector, getQ, SimTK::Stage::Position)

    OpenSim_DECLARE_OUTPUT(BodyAcc, SpatialVec, calcSpatialAcc,
            SimTK::Stage::Velocity)
    
    OpenSim_DECLARE_INPUT(input1, double, SimTK::Stage::Model, "");
    OpenSim_DECLARE_INPUT(AnglesIn, Vector, SimTK::Stage::Model, "");
    OpenSim_DECLARE_INPUT(fiberLength, double, SimTK::Stage::Model, "");
    OpenSim_DECLARE_INPUT(activation, double, SimTK::Stage::Model, "");

    Foo() : Component() {
        constructInfrastructure();
        m_ctr = 0;
        m_mutableCtr = 0;
    }

    double getSomething(const SimTK::State& state) const {
        const_cast<Foo *>(this)->m_ctr++;
        m_mutableCtr++;

        return state.getTime();
    }

    SimTK::Vec3 calcSomething(const SimTK::State& state) const {
        const_cast<Foo *>(this)->m_ctr++;
        m_mutableCtr++;

        double t = state.getTime();
        return SimTK::Vec3(t, t*t, sqrt(t));
    }

    SimTK::Vector getQ(const SimTK::State& state) const {
        return state.getQ();
    }

    SimTK::SpatialVec calcSpatialAcc(const SimTK::State& state) const {
        const_cast<Foo *>(this)->m_ctr++;
        m_mutableCtr++;

        return getSystem().getMatterSubsystem().getMobilizedBody(bindex)
            .getBodyAcceleration(state);
    }

protected:
    /** Component Interface */
    void extendConnect(Component& root) override {
        Super::extendConnect(root);
        // do any internal wiring
        world = dynamic_cast<TheWorld*>(&root);
    }

    void extendAddToSystem(MultibodySystem &system) const override {
        Super::extendAddToSystem(system);

        SimbodyMatterSubsystem& matter = system.updMatterSubsystem();

        Vec3 mInB(0.0, 1.0, 0);
        Vec3 mInP(0, 0, 0);

        SimTK::Body::Rigid bone(
            MassProperties(1, Vec3(0), Inertia::brick(0.5, 1, 0.5)));

        // Thigh connected by hip
        MobilizedBody::Pin b1ToGround(matter.updGround(), SimTK::Transform(mInP),
            bone, SimTK::Transform(mInB));

        //Pin knee connects shank
        MobilizedBody::Pin b1ToB2(b1ToGround, SimTK::Transform(mInP),
            bone, SimTK::Transform(mInB));

        bindex = b1ToB2.getMobilizedBodyIndex();
    }

private:
    int m_ctr;
    mutable int m_mutableCtr;


    void constructProperties() override {
        constructProperty_mass(1.0);
        Array<double> inertia(0.001, 6);
        inertia[0] = inertia[1] = inertia[2] = 0.1;
        constructProperty_inertia(inertia);
    }

    void constructOutputs() override {


    }

    // Keep indices and reference to the world
    mutable MobilizedBodyIndex bindex;
    ReferencePtr<TheWorld> world;

}; // End of class Foo

class Bar : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(Bar, Component);
public:

    double copytesting2 = 5;
    OpenSim_DECLARE_OUTPUT(copytesting, size_t, myself, SimTK::Stage::Model);
    OpenSim_DECLARE_OUTPUT(copytesting2, double, getCopytesting2,
                           SimTK::Stage::Model);

    OpenSim_DECLARE_OUTPUT(PotentialEnergy, double, getPotentialEnergy,
            SimTK::Stage::Velocity);

    OpenSim_DECLARE_OUTPUT_FOR_STATE_VARIABLE(fiberLength);
    OpenSim_DECLARE_OUTPUT_FOR_STATE_VARIABLE(activation);

    Bar() : Component() { constructInfrastructure(); }

    double getPotentialEnergy(const SimTK::State& state) const {
        const GeneralForceSubsystem& forces = world->getForceSubsystem();
        const Force& force = forces.getForce(fix);
        const Force::TwoPointLinearSpring& spring = 
            Force::TwoPointLinearSpring::downcast(force);
    
        return spring.calcPotentialEnergyContribution(state);
    }
    
    /** Returns the `this` pointer. Used to ensure that the std::function 
     within Outputs is properly copied when copying components. */
    size_t myself(const SimTK::State& s) const { return size_t(this); }
    
    double getCopytesting2(const SimTK::State& s) const { return copytesting2; }

protected:
    /** Component Interface */
    void extendConnect(Component& root) override{
        Super::extendConnect(root);
        // do any internal wiring
        world = dynamic_cast<TheWorld*>(&root);
        // perform custom checking
        if (updConnector<Foo>("parentFoo").getConnectee()
                == updConnector<Foo>("childFoo").getConnectee()){
            string msg = "ERROR - Bar::extendConnect()\n";
            msg += " parentFoo and childFoo cannot be the same component.";
            throw OpenSim::Exception(msg);
        }
    }

    // Copied here from Component for testing purposes.


    void extendAddToSystem(MultibodySystem& system) const override{

        GeneralForceSubsystem& forces = world->updForceSubsystem();
        SimbodyMatterSubsystem& matter = world->updMatterSubsystem();

        int nb = matter.getNumBodies();
        if (nb > 2) {
            const MobilizedBody& b1 = matter.getMobilizedBody(MobilizedBodyIndex(1));
            const MobilizedBody& b2 = matter.getMobilizedBody(MobilizedBodyIndex(2));

            Force::TwoPointLinearSpring 
                spring(forces, b1, Vec3(0.5,0,0), b2, Vec3(0.5,0,0), 10.0, 0.1);
            fix = spring.getForceIndex();
        }

        // We use these to test the Output's that are generated when we
        // add a StateVariable.
        addStateVariable("fiberLength", SimTK::Stage::Velocity);
        addStateVariable("activation", SimTK::Stage::Dynamics);

        // Create a hidden state variable, so we can ensure that hidden state
        // variables do not have a corresponding Output.
        bool hidden = true;
        addStateVariable("hiddenStateVar", SimTK::Stage::Dynamics, hidden);
    }

    void computeStateVariableDerivatives(const SimTK::State& state) const override {
        setStateVariableDerivativeValue(state, "fiberLength", 2.0);
        setStateVariableDerivativeValue(state, "activation", 3.0 * state.getTime());
        setStateVariableDerivativeValue(state, "hiddenStateVar", 
                                          exp(-0.5 * state.getTime()));
    }

private:
    void constructConnectors() override {
        constructConnector<Foo>("parentFoo");
        constructConnector<Foo>("childFoo");
    }

    // keep track of the force added by the component
    mutable ForceIndex fix;
    ReferencePtr<TheWorld> world;

}; // End of class Bar

// Create 2nd level derived class to verify that Component interface
// hold up.
class CompoundFoo : public Foo {
    OpenSim_DECLARE_CONCRETE_OBJECT(CompoundFoo, Foo);
public:
    //=============================================================================
    // PROPERTIES
    //=============================================================================
    OpenSim_DECLARE_PROPERTY(Foo1, Foo, "1st Foo of CompoundFoo");
    OpenSim_DECLARE_PROPERTY(Foo2, Foo, "2nd Foo of CompoundFoo");
    OpenSim_DECLARE_PROPERTY(scale1, double, "Scale factor for 1st Foo");
    OpenSim_DECLARE_PROPERTY(scale2, double, "Scale factor for 2nd Foo");

    CompoundFoo() : Foo() {
        constructInfrastructure();
    }

protected:
    // Component implementation interface
    void extendFinalizeFromProperties() override {
        // Allow Foo to do its finalize from properties
        Super::extendFinalizeFromProperties();

        // Mark components listed in properties as subcomponents
        Foo& foo1 = upd_Foo1();
        Foo& foo2 = upd_Foo2();

        // update CompoundFoo's properties based on it sub Foos
        double orig_mass = get_mass();
        upd_mass() = get_scale1()*foo1.get_mass() + get_scale2()*foo2.get_mass();

        double inertiaScale = (get_mass() / orig_mass);

        for (int i = 0; i < updProperty_inertia().size(); ++i) {
            upd_inertia(i) = inertiaScale*get_inertia(i);
        }
    }

private:
    void constructProperties() override {
        constructProperty_Foo1(Foo());
        constructProperty_Foo2(Foo());
        constructProperty_scale1(1.0);
        constructProperty_scale2(2.0);
    }   
}; // End of Class CompoundFoo

SimTK_NICETYPENAME_LITERAL(Foo);
SimTK_NICETYPENAME_LITERAL(Bar);

void testMisc() {
        // Define the Simbody system
        MultibodySystem system;

        TheWorld theWorld;
        theWorld.setName("World");
        
        // let component add its stuff to the system
        Foo& foo = *new Foo();
        foo.setName("Foo");
        theWorld.add(&foo);
        foo.set_mass(2.0);

        Foo* footTest = foo.clone();

        // bar0 is to test copying of the function within a component's outputs.
        std::unique_ptr<Bar> bar0(new Bar());
        Bar& bar = *bar0->clone();
        bar.copytesting2 = 6;
        bar.setName("Bar");
        theWorld.add(&bar);

        Bar barEqual(bar);
        barEqual = bar;

        //Configure the connector to look for its dependency by this name
        //Will get resolved and connected automatically at Component connect
        bar.updConnector<Foo>("parentFoo").set_connectee_name("Foo");
        bar.updConnector<Foo>("childFoo").connect(foo);
        
        // add a subcomponent
        // connect internals
        ASSERT_THROW( OpenSim::Exception,
                      theWorld.connect() );


        ComponentList<Component> worldTreeAsList = theWorld.getComponentList();
        std::cout << "list begin: " << worldTreeAsList.begin()->getName() << std::endl;
        for (auto it = worldTreeAsList.begin();
                  it != worldTreeAsList.end(); ++it) {
            std::cout << "Iterator is at: " << it->getName() << std::endl;
        }

        
        std::cout << "Using range-for loop: " << std::endl;
        for (const Component& component : worldTreeAsList) {
            std::cout << "Iterator is at: " << component.getName() << std::endl;
        }
        for (auto& component : worldTreeAsList) {
            std::cout << "Iterator is at: " << component.getName() << std::endl;
        }
        
        std::cout << "Iterate over only Foo's." << std::endl;
        for (auto& component : theWorld.getComponentList<Foo>()) {
            std::cout << "Iterator is at: " << component.getName() << std::endl;
        }
        

        Foo& foo2 = *new Foo();
        foo2.setName("Foo2");
        foo2.set_mass(3.0);

        theWorld.add(&foo2);

        bar.updConnector<Foo>("childFoo").set_connectee_name("Foo2");
        string connectorName = bar.updConnector<Foo>("childFoo").getConcreteClassName();

        // Bar should connect now
        theWorld.connect();

        std::cout << "Iterate over only Foo's." << std::endl;
        for (auto& component : theWorld.getComponentList<Foo>()) {
            std::cout << "Iterator is at: " << component.getName() << std::endl;
        }

        theWorld.buildUpSystem(system);

        const Foo& foo2found = theWorld.getComponent<Foo>("Foo2");
        ASSERT(foo2 == foo2found);

        // do any other input/output connections
        foo.updInput("input1").connect(bar.getOutput("PotentialEnergy"));

        // check how this model serializes
        string modelFile("testComponentInterfaceModel.osim");
        theWorld.print(modelFile);

        // Simbody model state setup
        State s = system.realizeTopology();

        int nu = system.getMatterSubsystem().getNumMobilities();

        //SimTK::Visualizer viz(system);
        //viz.drawFrameNow(s);
        const Vector q = Vector(s.getNQ(), SimTK::Pi/2);
        const Vector u = Vector(s.getNU(), 1.0);
        
        // Ensure the "this" pointer inside the output function is for the
        // correct Bar.
        system.realize(s, Stage::Model);
        std::cout << "bar0f " << std::endl;
        // Since bar0 is not part of any "world", we must call
        // finalizeFromProperties() on it ourselves in order to set the
        // "owner" of its outputs.
        bar0->finalizeFromProperties();
        // If bar's copytesting output is 0, then the following tests will pass
        // accidentally.
        SimTK_TEST(bar.getOutputValue<size_t>(s, "copytesting") != 0);
        // Make sure bar's outputs don't point to bar0.
        SimTK_TEST(bar.getOutputValue<size_t>(s, "copytesting") != size_t(bar0.get()));
        // Make sure bar's outputs are using bar underneath.
        SimTK_TEST(bar.getOutputValue<size_t>(s, "copytesting") == size_t(&bar));
        SimTK_TEST(bar0->getOutputValue<double>(s, "copytesting2") == 5);
        SimTK_TEST(bar.getOutputValue<double>(s, "copytesting2") == 6);
        
        // By deleting bar0 then calling getOutputValue on bar without a
        // segfault (throughout the remaining code), we ensure that bar
        // does not depend on bar0.
        bar0.reset(nullptr);


        for (int i = 0; i < 10; ++i){
            s.updTime() = i*0.01234;
            s.updQ() = (i+1)*q/10.0;
            system.realize(s, Stage::Velocity);

            const AbstractOutput& out1 = foo.getOutput("Output1");
            const AbstractOutput& out2 = foo.getOutput("Output2");
            const AbstractOutput& out3 = foo.getOutput("Qs");
            const AbstractOutput& out4 = foo.getOutput("BodyAcc");
            const AbstractOutput& out5 = bar.getOutput("PotentialEnergy");

            cout << "=========================[Time " << s.getTime() << "s]======================="<<endl;
            cout << out1.getName() <<"|"<< out1.getTypeName() <<"|"<< out1.getValueAsString(s) << endl;
            cout << out2.getName() <<"|"<< out2.getTypeName() <<"|"<< out2.getValueAsString(s) << endl;
            cout << out3.getName() <<"|"<< out3.getTypeName() <<"|"<< out3.getValueAsString(s) << endl;
            
            system.realize(s, Stage::Acceleration);
            cout << out4.getName() <<"|"<< out4.getTypeName() <<"|"<< out4.getValueAsString(s) << endl;
            cout << out5.getName() <<"|"<< out5.getTypeName() <<"|"<< out5.getValueAsString(s) << endl;

            //viz.report(s);
            system.realize(s, Stage::Report);

            cout << "foo.input1 = " << foo.getInputValue<double>(s, "input1") << endl;
        }

        MultibodySystem system2;
        TheWorld *world2 = new TheWorld(modelFile, true);
        
        world2->updComponent("Bar").getConnector<Foo>("childFoo");
        // We haven't called connect yet, so this connection isn't made yet.
        SimTK_TEST_MUST_THROW_EXC(
                world2->updComponent("Bar").getConnectee<Foo>("childFoo"),
                OpenSim::Exception
                 );

        ASSERT(theWorld == *world2, __FILE__, __LINE__,
            "Model serialization->deserialization FAILED");

        world2->setName("InternalWorld");
        world2->connect();

        world2->updComponent("Bar").getConnector<Foo>("childFoo");
        ASSERT("Foo2" ==
                world2->updComponent("Bar").getConnectee<Foo>("childFoo").getName());

        world2->buildUpSystem(system2);
        s = system2.realizeTopology();

        world2->print("clone_" + modelFile);

        // Test copy assignment
        TheWorld world3;
        world3 = *world2;
        world3.finalizeFromProperties();
        world3.getComponent("Bar").getConnector<Foo>("parentFoo");

        ASSERT(world3 == (*world2), __FILE__, __LINE__, 
            "Model copy assignment FAILED");

        // Add second world as the internal model of the first
        theWorld.add(world2);
        theWorld.connect();

        Bar& bar2 = *new Bar();
        bar2.setName("bar2");
        CompoundFoo& compFoo = *new CompoundFoo();
        compFoo.setName("BigFoo");

        compFoo.set_Foo1(foo);
        compFoo.set_Foo2(foo2);
        compFoo.finalizeFromProperties();

        world3.add(&compFoo);
        world3.add(&bar2);

        //Configure the connector to look for its dependency by this name
        //Will get resolved and connected automatically at Component connect
        bar2.updConnector<Foo>("parentFoo").set_connectee_name("BigFoo");
        bar2.updConnector<Foo>("childFoo").set_connectee_name("Foo");

        //world3.connect();
        world3.print("Compound_" + modelFile);

        MultibodySystem system3;
        theWorld.buildUpSystem(system3);
        //SimTK::Visualizer viz2(system2);

        // Connect our state variables.
        foo.updInput("fiberLength").connect(bar.getOutput("fiberLength"));
        foo.updInput("activation").connect(bar.getOutput("activation"));
        // Since hiddenStateVar is a hidden state variable, it has no
        // corresponding output.
        ASSERT_THROW( OpenSim::Exception,
            const AbstractOutput& out = bar.getOutput("hiddenStateVar") );

        s = system3.realizeTopology();

        bar.setStateVariableValue(s, "fiberLength", 1.5);
        bar.setStateVariableValue(s, "activation", 0);

        int nu3 = system3.getMatterSubsystem().getNumMobilities();

        // realize simbody system to velocity stage
        system3.realize(s, Stage::Velocity);

        RungeKuttaFeldbergIntegrator integ(system3);
        integ.setAccuracy(1.0e-3);

        TimeStepper ts(system3, integ);
        ts.initialize(s);
        ts.stepTo(1.0);
        s = ts.getState();

        // Check the result of the integration on our state variables.
        ASSERT_EQUAL(3.5, bar.getOutputValue<double>(s, "fiberLength"), 1e-10);
        ASSERT_EQUAL(1.5, bar.getOutputValue<double>(s, "activation"), 1e-10);

        // Ensure the connection works.
        ASSERT_EQUAL(3.5, foo.getInputValue<double>(s, "fiberLength"), 1e-10);
        ASSERT_EQUAL(1.5, foo.getInputValue<double>(s, "activation"), 1e-10);

        theWorld.dumpSubcomponents();

        theWorld.print("Doubled" + modelFile);
}

template <typename T>
class ConsoleReporter : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(ConsoleReporter, Component);
    // TODO interval
    // TODO constant interval reporting
    // TODO num significant digits (override).
    OpenSim_DECLARE_LIST_INPUT(input, T, SimTK::Stage::Acceleration,
                               "Quantities to print to console.");
public:
    ConsoleReporter() {
        constructInfrastructure();
    }
private:
    void extendRealizeReport(const State& state) const override {
        // multi input: loop through multi-inputs.
        // Output::getNumberOfSignificantDigits().
        // TODO print column names every 10 outputs.
        // TODO test by capturing stdout.
        // TODO prepend each line with "[<name>]" or "[reporter]" if no name is given.
        // TODO an actual implementation should do a static cast, since we
        // know that T is correct.
        const auto& input = getInput<T>("input");
        
        if (_printCount % 20 == 0) {
            std::cout << "[" << getName() << "] "
                      << std::setw(_width) << "time" << "| ";
            for (auto idx = 0; idx < input.getNumConnectees(); ++idx) {
                const auto& chan = Input<T>::downcast(input).getChannel(idx);
                const auto& outName = chan.getName();
                const auto& truncName = outName.size() <= _width ?
                    outName : outName.substr(outName.size() - _width);
                std::cout << std::setw(_width) << truncName << "|";
            }
            std::cout << "\n";
        }
        // TODO set width based on number of significant digits.
        std::cout << "[" << getName() << "] "
                  << std::setw(_width) << state.getTime() << "| ";
        for (const auto& chan : input.getChannels()) {
            const auto& value = chan->getValue(state);
            const auto& nSigFigs = chan->getOutput().getNumberOfSignificantDigits();
            std::cout << std::setw(_width)
                      << std::setprecision(nSigFigs) << value << "|";
        }
        std::cout << std::endl;
        
        const_cast<ConsoleReporter<T>*>(this)->_printCount++;
    }
    unsigned int _printCount = 0;
    int _width = 12;
};

void testListInputs() {
    MultibodySystem system;
    TheWorld theWorld;
    theWorld.setName("World");
    
    Foo& foo = *new Foo();
    foo.setName("Foo");
    theWorld.add(&foo);
    foo.set_mass(2.0);

    Foo& foo2 = *new Foo();
    foo2.setName("Foo2");
    foo2.set_mass(3.0);
    theWorld.add(&foo2);

    Bar& bar = *new Bar();
    bar.setName("Bar");
    theWorld.add(&bar);
    
    auto* reporter = new ConsoleReporter<double>();
    reporter->setName("rep0");
    theWorld.add(reporter);
    
    reporter->updInput("input").connect(foo.getOutput("Output1"));
    reporter->updInput("input").connect(bar.getOutput("PotentialEnergy"));
    reporter->updInput("input").connect(bar.getOutput("fiberLength"));
    reporter->updInput("input").connect(bar.getOutput("activation"));
    
    bar.updConnector<Foo>("parentFoo").set_connectee_name("Foo");
    bar.updConnector<Foo>("childFoo").set_connectee_name("Foo2");
    
    theWorld.connect();
    theWorld.buildUpSystem(system);
    
    State s = system.realizeTopology();
    
    const Vector q = Vector(s.getNQ(), SimTK::Pi/2);
    for (int i = 0; i < 10; ++i){
        s.updTime() = i*0.01234;
        s.updQ() = (i+1)*q/10.0;
        system.realize(s, Stage::Report);
    }
}




class HasListConnector : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(HasListConnector, Component);
public:
    HasListConnector() { constructInfrastructure(); }
    void constructConnectors() override {
        constructConnector<Foo>("foos", true);
    }
};

void testListConnectors() {
    MultibodySystem system;
    TheWorld theWorld;
    theWorld.setName("world");
    
    Foo& foo = *new Foo(); foo.setName("foo"); foo.set_mass(2.0);
    theWorld.add(&foo);

    Foo& foo2 = *new Foo(); foo2.setName("foo2"); foo2.set_mass(3.0);
    theWorld.add(&foo2);

    Bar& bar = *new Bar(); bar.setName("bar");
    theWorld.add(&bar);
    
    auto* haslist = new HasListConnector(); haslist->setName("haslist");
    theWorld.add(haslist);
    
    // Non-list connectors.
    bar.updConnector<Foo>("parentFoo").set_connectee_name("foo");
    bar.updConnector<Foo>("childFoo").set_connectee_name("foo2");
    
    // Ensure that calling connect() on bar's "parentFoo" doesn't increase
    // its number of connectees.
    bar.updConnector<Foo>("parentFoo").connect(foo);
    // TODO The "Already connected to 'foo'" is caught by `connect()`.
    SimTK_TEST(bar.getConnector<Foo>("parentFoo").getNumConnectees() == 1);
    
    // haslist is not connected, so there's an exception when connecting.
    // TODO or maybe list connectors are connected even if empty?
    // TODO maybe they just need to have paths to valid components to be connected.
    // haslist's "foos" is unconnected.
    // TODO SimTK_TEST_MUST_THROW_EXC(theWorld.connect(), OpenSim::Exception);
    // As it is now, it's OK for a list connector to have 0 connectees.
    theWorld.connect();
    
    // Now, configure haslist's connections.
    haslist->updConnector<Foo>("foos").connect(foo);
    
    // One connectee is all it needs to be connected.
    theWorld.connect();
    
    // But we can still add more connectees.
    haslist->updConnector<Foo>("foos").connect(foo2);
    SimTK_TEST(haslist->getConnector<Foo>("foos").getNumConnectees() == 2);
    
    theWorld.connect();
    theWorld.buildUpSystem(system);
    
    State s = system.realizeTopology();
    
    std::cout << bar.getConnectee<Foo>("parentFoo").get_mass() << std::endl;
    
    // TODO redo with the property list / the reference connect().
}

int main() {

    //Register new types for testing deserialization
    Object::registerType(Foo());
    Object::registerType(Bar());
    // Register connector objects that are in use
    Object::registerType(Connector<Foo>());
    Object::registerType(Connector<Bar>());

 // TODO   SimTK_START_TEST("testComponentIterface");
        SimTK_SUBTEST(testMisc);
        SimTK_SUBTEST(testListInputs);
        SimTK_SUBTEST(testListConnectors);
//    SimTK_END_TEST();
}
