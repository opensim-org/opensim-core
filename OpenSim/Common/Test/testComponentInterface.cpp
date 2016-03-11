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
        // Edit Sub 
        Sub& subc = updMemberSubcomponent<Sub>(intSubix);

        // add it the property list of components that owns and serializes them
        updProperty_components().adoptAndAppendValue(comp);
        try {
            finalizeFromProperties();
        }
        catch (ComponentAlreadyPartOfOwnershipTree& ex) {
            auto& compsProp = updProperty_components();
            //undo the adopt and append
            int ix = compsProp.findIndex(*comp);
            if (ix >= 0) {
                compsProp[ix].disconnect();
                // release the pointer to component in ix to undo adopt
                // erase pointer at ix from the property list to
            }
            throw ex;
        }
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
            const Sub& subc = getMemberSubcomponent<Sub>(intSubix);

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

    MemberSubcomponentIndex intSubix{ constructSubcomponent<Sub>("internalSub") };

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

    void constructInputs() override {
        constructInput<double>("input1", SimTK::Stage::Model);
        constructInput<Vector>("AnglesIn", SimTK::Stage::Model);

        constructInput<double>("fiberLength", SimTK::Stage::Model);
        constructInput<double>("activation", SimTK::Stage::Model);
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
        if (&updConnector<Foo>("parentFoo").getConnectee()
                == &updConnector<Foo>("childFoo").getConnectee()){
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
// holds up.
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

void testComponentPathNames();

int main() {

    //Register new types for testing deserialization
    Object::registerType(Foo());
    Object::registerType(Bar());
    // Register connector objects that are in use
    Object::registerType(Connector<Foo>());
    Object::registerType(Connector<Bar>());

    try {
        testComponentPathNames();

        // Define the Simbody system
        MultibodySystem system;

        TheWorld theWorld;
        theWorld.setName("World");
        theWorld.finalizeFromProperties();

        TheWorld* cloneWorld = theWorld.clone();
        cloneWorld->setName("ClonedWorld");
        cloneWorld->finalizeFromProperties();

        TheWorld copyWorld(theWorld);
        copyWorld.setName("CopiedWorld");
        copyWorld.finalizeFromProperties();

        const Sub& theSub = theWorld.getComponent<Sub>("internalSub");
        const Sub& cloneSub = cloneWorld->getComponent<Sub>("internalSub");
        const Sub& copySub = copyWorld.getComponent<Sub>("internalSub");

        // The clone and copy intern Sub components should be different
        // allocation (address) from original internal Sub
        ASSERT(&theSub != &cloneSub);
        ASSERT(&theSub != &copySub);
        // But their contents/values should be identical 
        ASSERT(theSub == cloneSub);
        ASSERT(theSub == copySub);

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
        ASSERT(barEqual == bar);

        //Configure the connector to look for its dependency by this name
        //Will get resolved and connected automatically at Component connect
        bar.updConnector<Foo>("parentFoo").set_connectee_name(foo.getFullPathName());
        bar.updConnector<Foo>("childFoo").connect(foo);
        
        // add a subcomponent
        // connect internals
        ASSERT_THROW( OpenSim::Exception,
                      theWorld.connect() );


        ComponentList<Component> worldTreeAsList = theWorld.getComponentList();
        std::cout << "list begin: " << worldTreeAsList.begin()->getName() << std::endl;
        for (ComponentList<Component>::const_iterator it = worldTreeAsList.begin();
            it != worldTreeAsList.end();
            ++it) {
            std::cout << "Iterator is at: " << it->getFullPathName() << std::endl;
        }

        
        std::cout << "Using range-for loop: " << std::endl;
        for (const Component& component : worldTreeAsList) {
            std::cout << "Iterator is at: " << component.getFullPathName() << std::endl;
        }

        
        std::cout << "Iterate over only Foo's." << std::endl;
        for (auto& component : theWorld.getComponentList<Foo>()) {
            std::cout << "Iterator is at: " << component.getFullPathName() << std::endl;
        }

        Foo& foo2 = *new Foo();
        foo2.setName("Foo2");
        foo2.set_mass(3.0);

        theWorld.add(&foo2);

        std::cout << "Iterate over Foo's after adding Foo2." << std::endl;
        for (auto& component : theWorld.getComponentList<Foo>()) {
            std::cout << "Iter at: " << component.getFullPathName() << std::endl;
        }

        bar.updConnector<Foo>("childFoo").connect(foo2);
        string connectorName = bar.updConnector<Foo>("childFoo").getConcreteClassName();

        // Bar should connect now
        theWorld.connect();
        theWorld.buildUpSystem(system);

        const Foo& foo2found = theWorld.getComponent<Foo>("Foo2");
        ASSERT(foo2 == foo2found);

        // do any other input/output connections
        foo.getInput("input1").connect(bar.getOutput("PotentialEnergy"));
    
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
        ASSERT_THROW( OpenSim::Exception,
                world2->updComponent("Bar").getConnectee<Foo>("childFoo") );

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

        ASSERT(&world3 != world2, __FILE__, __LINE__,
            "Model copy assignment FAILED: A copy was not made.");

        world3.finalizeFromProperties();

        ASSERT(world3 == *world2, __FILE__, __LINE__,
            "Model copy assignment FAILED: Property values are not identical.");

        world3.getComponent("Bar").getConnector<Foo>("parentFoo");

        auto& barInWorld3 = world3.getComponent<Bar>("Bar");
        auto& barInWorld2 = world2->getComponent<Bar>("Bar");
        ASSERT(&barInWorld3 != &barInWorld2, __FILE__, __LINE__, 
            "Model copy assignment FAILED: property was not copied but "
            "assigned the same memory");

        world3.setName("World3");

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
        bar2.updConnector<Foo>("parentFoo")
            .set_connectee_name(compFoo.getRelativePathName(bar2));
        bar2.updConnector<Foo>("childFoo").connect(foo);

        world3.finalizeFromProperties();
        world3.print("Compound_" + modelFile);

        cout << "Adding world3 to theWorld" << endl;
        theWorld.add(world3.clone());
        
        // TODO add the bar component again, which gets adopted by world3.
        // This must trigger an exception, which it does, but the problem 
        // is that the we don't have access to the property list to now
        // remove it. Uncommenting current results in a fault when the 
        // the destructor attempts to delete bar2 a second time around.
        // ASSERT_THROW( ComponentAlreadyPartOfOwnershipTree,
        //              world3.add(&bar2));

        cout << "Connecting theWorld:" << endl;
        theWorld.dumpSubcomponents();
        theWorld.connect();

        MultibodySystem system3;
        cout << "Building theWorld's system:" << endl;
        theWorld.buildUpSystem(system3);

        // Connect our state variables.
        foo.getInput("fiberLength").connect(bar.getOutput("fiberLength"));
        foo.getInput("activation").connect(bar.getOutput("activation"));
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

        std::cout << "Iterate over all Components in the world." << std::endl;
        for (auto& component : theWorld.getComponentList<Component>()) {
            std::cout << "Iterator is at: " << component.getFullPathName() << std::endl;
        }

        // Should fail to get Component when path is not specified
        ASSERT_THROW(OpenSim::Exception,
            theWorld.getComponent<CompoundFoo>("BigFoo") );

        // With path to the component it should work
        auto& bigFoo = theWorld.getComponent<CompoundFoo>("World/World3/BigFoo");
        const Sub& topSub = theWorld.getComponent<Sub>("InternalWorld/internalSub");
        
        // Should also be able to get top-level
        auto& topFoo = theWorld.getComponent<Foo>("Foo2");
        cout << "Top level Foo2 path name: " << topFoo.getFullPathName() << endl;

        // And the leaf Foo2 from BigFoo
        auto& leafFoo = bigFoo.getComponent<Foo>("Foo2");
        cout << "Leaf level Foo2 path name: " << leafFoo.getFullPathName() << endl;

        theWorld.print("Nested_" + modelFile);
    }
    catch (const std::exception& e) {
        cout << e.what() <<endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}


void testComponentPathNames()
{
    Foo foo;
    Bar bar;
    TheWorld top;

    // These are not valid component names
    // Only using for testing as surrogates for path names
    // which are computed by the component. Just testing
    // the relative path name facility here.
    top.setName("Top");
    foo.setName("A/B/C/D");
    bar.setName("A/B/E");

    std::string fooWrtBar = foo.getRelativePathName(bar);
    ASSERT(fooWrtBar == "../C/D"); // "/A/B/" as common

    std::string barWrtFoo= bar.getRelativePathName(foo);
    ASSERT(barWrtFoo == "../../E"); // "/A/B/" as common

    // null case foo wrt foo
    std::string fooWrtFoo = foo.getRelativePathName(foo);
    ASSERT(fooWrtFoo == "");

    std::string topFullPath = top.getFullPathName();
    std::string fooWrtTop = foo.getRelativePathName(top);
    ASSERT(fooWrtTop == "../A/B/C/D");

    std::string topWrtFoo = top.getRelativePathName(foo);
    ASSERT(topWrtFoo== "../../../../Top");

    foo.setName("World/Foo");
    bar.setName("World3/bar2");
    fooWrtBar = foo.getRelativePathName(bar);
    ASSERT(fooWrtBar == "../../World/Foo");

    foo.setName("World3/bar2/foo1");
    fooWrtBar = foo.getRelativePathName(bar);
    ASSERT(fooWrtBar == "./foo1");

    bar.setName("LegWithConstrainedFoot/footConstraint");
    foo.setName("LegWithConstrainedFoot/foot");
    barWrtFoo = bar.getRelativePathName(foo);
    ASSERT(barWrtFoo == "../footConstraint");

    // Now build use real components and assemble them 
    // into a tree and test the path names that are 
    // generated on the fly.
    TheWorld* A = new TheWorld();
    TheWorld* B = new TheWorld();
    TheWorld* C = new TheWorld();
    TheWorld* D = new TheWorld();
    TheWorld* E = new TheWorld();
    A->setName("A");
    B->setName("B");
    C->setName("C");
    D->setName("D");
    E->setName("E");
    
    top.add(A);
    A->add(B);
    B->add(C);
    A->add(D);
    D->add(E);

    top.dumpSubcomponents();

    std::string fullPathC = C->getFullPathName();
    ASSERT(fullPathC == "/Top/A/B/C");

    std::string fullPathE = E->getFullPathName();
    ASSERT(fullPathE == "/Top/A/D/E");

    // Must specify a unique path to E
    ASSERT_THROW(OpenSim::ComponentNotFoundOnSpecifiedPath,
        auto& eref = top.getComponent("E") );

    auto& cref = top.getComponent(fullPathC);
    auto& eref = top.getComponent(fullPathE);

    auto cFromE = cref.getRelativePathName(eref);
    ASSERT(cFromE == "../../B/C");

    auto eFromC = eref.getRelativePathName(cref);
    ASSERT(eFromC == "../../D/E");

    // verify that we can also navigate relative paths properly
    auto& eref2 = cref.getComponent(eFromC);
    ASSERT(eref2 == eref);

    Foo* foo1 = new Foo();
    foo1->setName("Foo1");
    Foo* foo2 = new Foo();
    foo2->setName("Foo2");
    Bar* bar2 = new Bar();
    bar2->setName("Bar2");

    A->add(foo1);
    A->add(foo2);
    A->add(bar2);

    TheWorld* F = A->clone();
    F->setName("F");
    top.add(F);

    top.dumpSubcomponents();

    std::string fFoo1FullPath = 
        F->getComponent<Foo>("Foo1").getFullPathName();
    std::string aBar2FullPath = 
        A->getComponent<Bar>("Bar2").getFullPathName();
    auto bar2FromBarFoo = 
        bar2->getRelativePathName(F->getComponent<Foo>("Foo1"));

    // Verify deep copy of subcomponents
    const Foo& foo1inA = top.getComponent<Foo>("/Top/A/Foo1");
    const Foo& foo1inF = top.getComponent<Foo>("/Top/F/Foo1");
    ASSERT(&foo1inA != &foo1inF);

    // double check that we have the original Foo foo1 in A
    ASSERT(&foo1inA == foo1);

    // This bar2 that belongs to A and connects the two foo2s
    bar2->updConnector<Foo>("parentFoo").connect(*foo2);
    bar2->updConnector<Foo>("childFoo")
        .connect(F->getComponent<Foo>("Foo2"));

    auto& foo2inF = bar2->getComponent<Foo>("../../F/Foo2");

    // now wire up bar2 that belongs to F and connect the 
    // two foo1s one in A and other F
    auto& fbar2 = F->updComponent<Bar>("Bar2");
    ASSERT(&fbar2 != bar2);

    fbar2.updConnector<Foo>("parentFoo").connect(*foo1);
    fbar2.updConnector<Foo>("childFoo")
        .set_connectee_name("../Foo1");

    top.dumpSubcomponents();
    top.connect();
}
