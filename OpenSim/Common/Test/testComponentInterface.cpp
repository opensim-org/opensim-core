/* -------------------------------------------------------------------------- *
 *                OpenSim:  testComponentInterface.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
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
#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Common/TableSource.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <simbody/internal/SimbodyMatterSubsystem.h>
#include <simbody/internal/GeneralForceSubsystem.h>
#include <simbody/internal/Force.h>
#include <simbody/internal/MobilizedBody_Pin.h>
#include <simbody/internal/MobilizedBody_Ground.h>

using namespace OpenSim;
using namespace std;
using namespace SimTK;

class Foo;
class Bar;

class Sub : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(Sub, Component);
public:
    OpenSim_DECLARE_OUTPUT_FOR_STATE_VARIABLE(subState);
    Sub() = default;
    virtual ~Sub() = default;
private:
    void extendAddToSystem(MultibodySystem &system) const override {
        Super::extendAddToSystem(system);
        addStateVariable("subState", Stage::Dynamics);
    }
    void computeStateVariableDerivatives(const SimTK::State& s) const override {
        double deriv = exp(-2.0*s.getTime());
        setStateVariableDerivativeValue(s, "subState", deriv);
    }
}; //end class Sub

class TheWorld : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(TheWorld, Component);
public:
    TheWorld() : Component() { }

    TheWorld(const std::string& fileName, bool updFromXMLNode = false)
        : Component(fileName, updFromXMLNode) {
        // Propagate XML file values to properties 
        updateFromXMLDocument();
        // add components listed as properties as sub components.
        finalizeFromProperties();
    }

    void add(Component* comp) {
        addComponent(comp);
        // Edit Sub 
        /*Sub& subc = */updMemberSubcomponent<Sub>(intSubix);
    }

    // Top level connection method for this all encompassing component, TheWorld
    void connect() {
        Super::connect(*this);
    }
    void buildUpSystem(MultibodySystem& system) { 
        connect();
        addToSystem(system);
    }

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
            // const Sub& subc = getMemberSubcomponent<Sub>(intSubix);

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

    OpenSim_DECLARE_OUTPUT(Output1, double, getSomething, SimTK::Stage::Time);
    OpenSim_DECLARE_OUTPUT(Output2, SimTK::Vec3, calcSomething,
            SimTK::Stage::Time);

    OpenSim_DECLARE_OUTPUT(Output3, double, getSomethingElse, SimTK::Stage::Time);

    OpenSim_DECLARE_OUTPUT(Qs, Vector, getQ, SimTK::Stage::Position);

    OpenSim_DECLARE_OUTPUT(BodyAcc, SpatialVec, calcSpatialAcc,
            SimTK::Stage::Velocity);

    OpenSim_DECLARE_OUTPUT(return_by_ref, double, getReturnByRef,
            SimTK::Stage::Time);

    OpenSim_DECLARE_INPUT(input1, double, SimTK::Stage::Model, "");
    OpenSim_DECLARE_INPUT(AnglesIn, Vector, SimTK::Stage::Model, "");
    OpenSim_DECLARE_INPUT(fiberLength, double, SimTK::Stage::Model, "");
    OpenSim_DECLARE_INPUT(activation, double, SimTK::Stage::Model, "");
    OpenSim_DECLARE_LIST_INPUT(listInput1, double, SimTK::Stage::Model, "");

    Foo() : Component() {
        constructProperties();
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

    double getSomethingElse(const SimTK::State& state) const {
        return 1.618;
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

    const double& getReturnByRef(const SimTK::State& s) const {
        // Must return something that is stored in the state!
        return s.getTime();
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


    void constructProperties() {
        constructProperty_mass(1.0);
        Array<double> inertia(0.001, 6);
        inertia[0] = inertia[1] = inertia[2] = 0.1;
        constructProperty_inertia(inertia);
    }

    // Keep indices and reference to the world
    mutable MobilizedBodyIndex bindex;
    ReferencePtr<TheWorld> world;

}; // End of class Foo

class Bar : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(Bar, Component);
public:
    
    OpenSim_DECLARE_CONNECTOR(parentFoo, Foo, "");
    OpenSim_DECLARE_CONNECTOR(childFoo, Foo, "");

    // This is used to test output copying and returns the address of the 
    // component.
    OpenSim_DECLARE_OUTPUT(copytesting, size_t, myself, SimTK::Stage::Model);
    // Use this member variable to ensure that output functions get copied
    // correctly.
    double copytestingViaMemberVariable = 5;
    OpenSim_DECLARE_OUTPUT(copytestingMemVar, double, getCopytestingMemVar,
                           SimTK::Stage::Model);

    OpenSim_DECLARE_OUTPUT(PotentialEnergy, double, getPotentialEnergy,
            SimTK::Stage::Velocity);

    OpenSim_DECLARE_OUTPUT_FOR_STATE_VARIABLE(fiberLength);
    OpenSim_DECLARE_OUTPUT_FOR_STATE_VARIABLE(activation);

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
    
    double getCopytestingMemVar(const SimTK::State& s) const
    { return copytestingViaMemberVariable; }

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
        constructProperties();
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
    void constructProperties() {
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
    theWorld.finalizeFromProperties();

    // ComponentHasNoSystem exception should be thrown if user attempts to read
    // or write state, discrete, or cache variables before Component has an
    // underlying MultibodySystem.
    {
        SimTK::State sBlank;
        const std::string varName = "waldo"; //dummy name

        ASSERT_THROW(ComponentHasNoSystem, theWorld.findStateVariable(varName));
        ASSERT_THROW(ComponentHasNoSystem, theWorld.getNumStateVariables());
        ASSERT_THROW(ComponentHasNoSystem, theWorld.getStateVariableNames());
        ASSERT_THROW(ComponentHasNoSystem,
            theWorld.getStateVariableValue(sBlank, varName));
        ASSERT_THROW(ComponentHasNoSystem,
            theWorld.setStateVariableValue(sBlank, varName, 0.));
        ASSERT_THROW(ComponentHasNoSystem,
            theWorld.getStateVariableValues(sBlank));
        ASSERT_THROW(ComponentHasNoSystem,
            theWorld.setStateVariableValues(sBlank, SimTK::Vector(1, 0.)));
        ASSERT_THROW(ComponentHasNoSystem,
            theWorld.getStateVariableDerivativeValue(sBlank, varName));
        ASSERT_THROW(ComponentHasNoSystem,
            theWorld.getDiscreteVariableValue(sBlank, varName));
        ASSERT_THROW(ComponentHasNoSystem,
            theWorld.setDiscreteVariableValue(sBlank, varName, 0.));
        ASSERT_THROW(ComponentHasNoSystem,
            theWorld.getCacheVariableValue<double>(sBlank, varName));
        ASSERT_THROW(ComponentHasNoSystem,
            theWorld.setCacheVariableValue(sBlank, varName, 0.));
        ASSERT_THROW(ComponentHasNoSystem,
            theWorld.updCacheVariableValue<double>(sBlank, varName));
        ASSERT_THROW(ComponentHasNoSystem,
            theWorld.markCacheVariableValid(sBlank, varName));
        ASSERT_THROW(ComponentHasNoSystem,
            theWorld.markCacheVariableInvalid(sBlank, varName));
        ASSERT_THROW(ComponentHasNoSystem,
            theWorld.isCacheVariableValid(sBlank, varName));
    }

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

    // Foo* footTest = foo.clone();

    // bar0 is to test copying of the function within a component's outputs.
    std::unique_ptr<Bar> bar0(new Bar());
    Bar& bar = *bar0->clone();
    bar.copytestingViaMemberVariable = 6;
    bar.setName("Bar");
    theWorld.add(&bar);

    Bar barEqual(bar);
    ASSERT(barEqual == bar);

    //Configure the connector to look for its dependency by this name
    //Will get resolved and connected automatically at Component connect
    bar.updConnector<Foo>("parentFoo").setConnecteeName(foo.getAbsolutePathName());
    bar.updConnector<Foo>("childFoo").connect(foo);
        
    // connect internals
    // parentFoo and childFoo are not allowed to be the same component.
    ASSERT_THROW( OpenSim::Exception,
                  theWorld.connect() );


    auto worldTreeAsList = theWorld.getComponentList();
    std::cout << "list begin: " << worldTreeAsList.begin()->getName() << std::endl;
    for (auto it = worldTreeAsList.begin();
              it != worldTreeAsList.end(); ++it) {
        std::cout << "Iterator is at: " << it->getAbsolutePathName() << std::endl;
    }

        
    std::cout << "Using range-for loop: " << std::endl;
    for (const Component& component : worldTreeAsList) {
        std::cout << "Iterator is at: " << component.getAbsolutePathName() << std::endl;
    }

        
    std::cout << "Iterate over only Foo's." << std::endl;
    for (auto& component : theWorld.getComponentList<Foo>()) {
        std::cout << "Iterator is at: " << component.getAbsolutePathName() << std::endl;
    }

    Foo& foo2 = *new Foo();
    foo2.setName("Foo2");
    foo2.set_mass(3.0);

    theWorld.add(&foo2);

    std::cout << "Iterate over Foo's after adding Foo2." << std::endl;
    for (auto& component : theWorld.getComponentList<Foo>()) {
        std::cout << "Iter at: " << component.getAbsolutePathName() << std::endl;
    }

    // Query existing components.
    theWorld.printComponentsMatching("");
    SimTK_TEST(theWorld.hasComponent("Foo"));
    SimTK_TEST(!theWorld.hasComponent("Nonexistant"));
    SimTK_TEST(theWorld.hasComponent<Foo>("Foo"));
    SimTK_TEST(!theWorld.hasComponent<Bar>("Foo"));
    SimTK_TEST(!theWorld.hasComponent<Foo>("Nonexistant"));


    bar.updConnector<Foo>("childFoo").connect(foo2);
    string connectorName = bar.updConnector<Foo>("childFoo").getName();

    // Bar should connect now
    theWorld.connect();
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

    // int nu = system.getMatterSubsystem().getNumMobilities();

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
    SimTK_TEST(bar0->getOutputValue<double>(s, "copytestingMemVar") == 5);
    SimTK_TEST(bar.getOutputValue<double>(s, "copytestingMemVar") == 6);
        
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

    // Test the output that returns by const T&.
    SimTK_TEST(foo.getOutputValue<double>(s, "return_by_ref") == s.getTime());

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

    // setting Foo's creates copies that are now part of CompoundFoo
    compFoo.set_Foo1(foo);
    compFoo.set_Foo2(foo2);
    compFoo.finalizeFromProperties();

    world3.add(&compFoo);
    world3.add(&bar2);

    //Configure the connector to look for its dependency by this name
    //Will get resolved and connected automatically at Component connect
    bar2.updConnector<Foo>("parentFoo")
    .setConnecteeName(compFoo.getRelativePathName(bar2));
    
    bar2.updConnector<Foo>("childFoo").connect(foo);
    compFoo.upd_Foo1().updInput("input1")
        .connect(bar2.getOutput("PotentialEnergy"));

    world3.finalizeFromProperties();
    world3.print("Compound_" + modelFile);

    cout << "Adding world3 to theWorld" << endl;
    theWorld.add(world3.clone());

    // Should not be able to add the same Component twice within the same tree
    ASSERT_THROW( ComponentAlreadyPartOfOwnershipTree,
                  world3.add(&bar2));

    cout << "Connecting theWorld:" << endl;
    theWorld.dumpSubcomponents();
    theWorld.finalizeFromProperties();
    theWorld.connect();

    auto* reporter = new TableReporterVector();
    reporter->set_report_time_interval(0.1);
    reporter->updInput("inputs").connect(foo.getOutput("Qs"));
    theWorld.add(reporter);

    MultibodySystem system3;
    cout << "Building theWorld's system:" << endl;
    theWorld.buildUpSystem(system3);

    // Connect our state variables.
    foo.updInput("fiberLength").connect(bar.getOutput("fiberLength"));
    foo.updInput("activation").connect(bar.getOutput("activation"));
    // Since hiddenStateVar is a hidden state variable, it has no
    // corresponding output.
    ASSERT_THROW( OpenSim::Exception,
          /*const AbstractOutput& out = */bar.getOutput("hiddenStateVar") );

    s = system3.realizeTopology();

    bar.setStateVariableValue(s, "fiberLength", 1.5);
    bar.setStateVariableValue(s, "activation", 0);

    // int nu3 = system3.getMatterSubsystem().getNumMobilities();

    // realize simbody system to velocity stage
    system3.realize(s, Stage::Velocity);

    RungeKuttaFeldbergIntegrator integ(system3);
    integ.setAccuracy(1.0e-3);

    TimeStepper ts(system3, integ);
    ts.initialize(s);
    ts.stepTo(1.0);
    s = ts.getState();

    // realize simbody system to velocity stage
    system3.realize(s, Stage::Velocity);

    // Get the results of integrating the system forward
    const TimeSeriesTable_<Real>& results = reporter->getTable();
    ASSERT(results.getNumRows() == 11, __FILE__, __LINE__,
        "Number of rows in Reporter results not equal to number of time intervals.");
    cout << "************** Contents of Table of Results ****************" << endl;
    cout << results << endl;
    cout << "***************** Qs Output at Final state *****************" << endl;
    auto& finalVal = foo.getOutputValue<Vector>(s, "Qs");
    (~finalVal).dump();
    size_t ncols = results.getNumColumns();
    ASSERT(ncols == static_cast<size_t>(finalVal.size()), __FILE__, __LINE__,
        "Number of cols in Reporter results not equal to size of Output'Qs' size.");

    // Check the result of the integration on our state variables.
    ASSERT_EQUAL(3.5, bar.getOutputValue<double>(s, "fiberLength"), 1e-10);
    ASSERT_EQUAL(1.5, bar.getOutputValue<double>(s, "activation"), 1e-10);

    // Ensure the connection works.
    ASSERT_EQUAL(3.5, foo.getInputValue<double>(s, "fiberLength"), 1e-10);
    ASSERT_EQUAL(1.5, foo.getInputValue<double>(s, "activation"), 1e-10);

    theWorld.dumpSubcomponents();

    std::cout << "Iterate over all Components in the world." << std::endl;
    for (auto& component : theWorld.getComponentList<Component>()) {
        std::cout << "Iterator is at: " << component.getAbsolutePathName() << std::endl;
    }

    // Should fail to get Component when path is not specified
    ASSERT_THROW(OpenSim::Exception,
        theWorld.getComponent<CompoundFoo>("BigFoo") );

    // With path to the component it should work
    auto& bigFoo = theWorld.getComponent<CompoundFoo>("World/World3/BigFoo");
    // const Sub& topSub = theWorld.getComponent<Sub>("InternalWorld/internalSub");
        
    // Should also be able to get top-level
    auto& topFoo = theWorld.getComponent<Foo>("Foo2");
    cout << "Top level Foo2 path name: " << topFoo.getAbsolutePathName() << endl;

    // And the leaf Foo2 from BigFoo
    auto& leafFoo = bigFoo.getComponent<Foo>("Foo2");
    cout << "Leaf level Foo2 path name: " << leafFoo.getAbsolutePathName() << endl;

    theWorld.print("Nested_" + modelFile);
}



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

    bar.updConnector<Foo>("parentFoo").setConnecteeName("Foo");
    bar.updConnector<Foo>("childFoo").setConnecteeName("Foo2");
    
    auto* reporter = new ConsoleReporter();
    reporter->setName("rep0");
    theWorld.add(reporter);

    // wire up console reporter inputs to desired model outputs
    reporter->updInput("inputs").connect(foo.getOutput("Output1"));
    reporter->updInput("inputs").connect(bar.getOutput("PotentialEnergy"));
    reporter->updInput("inputs").connect(bar.getOutput("fiberLength"));
    reporter->updInput("inputs").connect(bar.getOutput("activation"));

    auto* tabReporter = new TableReporter();
    tabReporter->setName("TableReporterMixedOutputs");
    theWorld.add(tabReporter);

    // wire up table reporter inputs (using convenience method) to desired 
    // model outputs
    tabReporter->updInput().connect(bar.getOutput("fiberLength"));
    tabReporter->updInput().connect(bar.getOutput("activation"));
    tabReporter->updInput().connect(foo.getOutput("Output1"));
    tabReporter->updInput().connect(bar.getOutput("PotentialEnergy"));

    theWorld.connect();
    theWorld.buildUpSystem(system);
    
    State s = system.realizeTopology();
    
    const Vector q = Vector(s.getNQ(), SimTK::Pi/2);
    for (int i = 0; i < 10; ++i){
        s.updTime() = i*0.01234;
        s.updQ() = (i+1)*q/10.0;
        system.realize(s, Stage::Report);
    }

    cout << "  TableReporterMixedOutputs (contents)" << endl;
    cout << tabReporter->getTable() << endl;

    tabReporter->clearTable();
    ASSERT(tabReporter->getTable().getNumRows() == 0);
}


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
    
    // Non-list connectors.
    bar.updConnector<Foo>("parentFoo").setConnecteeName("foo");
    bar.updConnector<Foo>("childFoo").setConnecteeName("foo2");
    
    // Ensure that calling connect() on bar's "parentFoo" doesn't increase
    // its number of connectees.
    bar.updConnector<Foo>("parentFoo").connect(foo);
    // TODO The "Already connected to 'foo'" is caught by `connect()`.
    SimTK_TEST(bar.getConnector<Foo>("parentFoo").getNumConnectees() == 1);
    
    theWorld.connect();
    theWorld.buildUpSystem(system);
    
    State s = system.realizeTopology();
    
    std::cout << bar.getConnectee<Foo>("parentFoo").get_mass() << std::endl;
    
    // TODO redo with the property list / the reference connect().
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

    std::string topAbsPath = top.getAbsolutePathName();
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
    ASSERT(fooWrtBar == "foo1");

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

    std::string absPathC = C->getAbsolutePathName();
    ASSERT(absPathC == "/Top/A/B/C");

    std::string absPathE = E->getAbsolutePathName();
    ASSERT(absPathE == "/Top/A/D/E");

    // Must specify a unique path to E
    ASSERT_THROW(OpenSim::ComponentNotFoundOnSpecifiedPath,
                 /*auto& eref = */top.getComponent("E") );

    auto& cref = top.getComponent(absPathC);
    auto& eref = top.getComponent(absPathE);

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

    std::string fFoo1AbsPath = 
        F->getComponent<Foo>("Foo1").getAbsolutePathName();
    std::string aBar2AbsPath = 
        A->getComponent<Bar>("Bar2").getAbsolutePathName();
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

    // auto& foo2inF = bar2->getComponent<Foo>("../../F/Foo2");

    // now wire up bar2 that belongs to F and connect the 
    // two foo1s one in A and other F
    auto& fbar2 = F->updComponent<Bar>("Bar2");
    ASSERT(&fbar2 != bar2);

    fbar2.updConnector<Foo>("parentFoo").connect(*foo1);
    fbar2.updConnector<Foo>("childFoo")
        .setConnecteeName("../Foo1");

    top.dumpSubcomponents();
    top.connect();
}

void testInputOutputConnections()
{
    TheWorld world;
    Foo* foo1 = new Foo();
    Foo* foo2 = new Foo();
    Bar* bar = new Bar();

    foo1->setName("foo1");
    foo2->setName("foo2");
    bar->setName("bar");
    bar->updConnector<Foo>("parentFoo").connect(*foo1);
    bar->updConnector<Foo>("childFoo").connect(*foo2);
    
    world.add(foo1);
    world.add(foo2);
    world.add(bar);

    MultibodySystem mbs;

    // Should yield warnings about unconnected Inputs
    cout << "Unsatisfied Inputs during world.connect()" << endl;
    world.connect();

    // do any other input/output connections
    foo1->updInput("input1").connect(bar->getOutput("PotentialEnergy"));

    // Test various exceptions for inputs, outputs, connectors
    ASSERT_THROW(InputNotFound, foo1->getInput("input0"));
    ASSERT_THROW(ConnectorNotFound, bar->updConnector<Foo>("parentFoo0"));
    ASSERT_THROW(OutputNotFound, 
        world.getComponent("./internalSub").getOutput("subState0"));
    // Ensure that getOutput does not perform a "find"
    ASSERT_THROW(OutputNotFound,
        world.getOutput("./internalSub/subState"));

    foo2->updInput("input1").connect(world.getComponent("./internalSub").getOutput("subState"));

    foo1->updInput("AnglesIn").connect(foo2->getOutput("Qs"));
    foo2->updInput("AnglesIn").connect(foo1->getOutput("Qs"));

    foo1->updInput("activation").connect(bar->getOutput("activation"));
    foo1->updInput("fiberLength").connect(bar->getOutput("fiberLength"));

    foo2->updInput("activation").connect(bar->getOutput("activation"));
    foo2->updInput("fiberLength").connect(bar->getOutput("fiberLength"));

    cout << "Unsatisfied Inputs after wiring Outputs to Inputs." << endl;
    world.connect();
    world.buildUpSystem(mbs);
}

void testInputConnecteeNames() {
    auto testPath = [](const std::string& path,
                       const std::string& compPath,
                       const std::string& outputName,
                       const std::string& channelName,
                       const std::string& alias) {
        ChannelPath p(path);
        SimTK_TEST(p.toString() == path); // roundtrip.
        SimTK_TEST(p.getComponentPath().toString() == compPath);
        SimTK_TEST(p.getOutputName() == outputName);
        SimTK_TEST(p.getChannelName() == channelName);
        SimTK_TEST(p.getAlias() == alias);
    };
    testPath("/foo/bar/output", "/foo/bar", "output", "", "");
    testPath("/foo/bar/output:channel", "/foo/bar", "output", "channel", "");
    testPath("/foo/bar/output(baz)", "/foo/bar", "output", "", "baz");
    testPath("/foo/bar/output:channel(baz)",
             "/foo/bar", "output", "channel", "baz");
    
    // TODO test relative paths.
    // TODO move to same test file as tests for ComponentPath.
    
    // TODO remove parseConnecteeName().
    std::string outputPath, channelName, alias;
    
    AbstractInput::parseConnecteeName("/foo/bar/output",
                                      outputPath, channelName, alias);
    SimTK_TEST(outputPath == "/foo/bar/output");
    SimTK_TEST(channelName == "");
    SimTK_TEST(alias == "");
    
    AbstractInput::parseConnecteeName("/foo/bar/output:channel",
                                      outputPath, channelName, alias);
    SimTK_TEST(outputPath == "/foo/bar/output");
    SimTK_TEST(channelName == "channel");
    SimTK_TEST(alias == "");
    
    AbstractInput::parseConnecteeName("/foo/bar/output(baz)",
                                      outputPath, channelName, alias);
    SimTK_TEST(outputPath == "/foo/bar/output");
    SimTK_TEST(channelName == "");
    SimTK_TEST(alias == "baz");
    
    AbstractInput::parseConnecteeName("/foo/bar/output:channel(baz)",
                                      outputPath, channelName, alias);
    SimTK_TEST(outputPath == "/foo/bar/output");
    SimTK_TEST(channelName == "channel");
    SimTK_TEST(alias == "baz");
    
    // TODO test just an output name, or just a channel name, with no Component path.
    // TODO test invalid names as well.
}


void testExceptionsForConnecteeTypeMismatch() {
    // Create Component classes for use in the following tests.
    // --------------------------------------------------------
    // This class has Outputs.
    class A : public Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(A, Component);
    public:
        OpenSim_DECLARE_OUTPUT(out1, double, calcOut1, SimTK::Stage::Time);
        OpenSim_DECLARE_LIST_OUTPUT(outL, double, calcOutL, SimTK::Stage::Time);
        double calcOut1(const SimTK::State& state) const { return 0; }
        double calcOutL(const SimTK::State& state,
                        const std::string& channel) const { return 0; }
    private:
        void extendFinalizeFromProperties() override {
            // Register Channels for list output 'outL'.
            auto& outL = updOutput("outL");
            outL.addChannel("0"); outL.addChannel("1"); outL.addChannel("2");
        }
    };
    // This class has Inputs.
    class B : public Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(B, Component);
    public:
        OpenSim_DECLARE_INPUT(in1, Vec3, SimTK::Stage::Model, "");
        OpenSim_DECLARE_LIST_INPUT(inL, Vec3, SimTK::Stage::Model, "");
    };
    // This class has a connector.
    class C : public Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(C, Component);
    public:
        OpenSim_DECLARE_CONNECTOR(conn1, A, "");
    };
    
    // Test various type mismatches.
    // -----------------------------
    // First, check for exceptions when directly connecting inputs to outputs
    // (or connectors to connectees).
    { // Connector.
        TheWorld model;
        B* b = new B(); b->setName("b");
        C* c = new C(); c->setName("c");
        model.add(b); model.add(c);
        SimTK_TEST_MUST_THROW_EXC(c->updConnector("conn1").connect(*b),
                                  OpenSim::Exception);
    }
    { // single-value output -> single-value input.
        TheWorld model;
        A* a = new A(); a->setName("a");
        B* b = new B(); b->setName("b");
        model.add(a); model.add(b);
        SimTK_TEST_MUST_THROW_EXC(b->updInput("in1").connect(a->getOutput("out1")),
                                  OpenSim::Exception);
    }
    { // single-value output -> list input.
        TheWorld model;
        A* a = new A(); a->setName("a");
        B* b = new B(); b->setName("b");
        model.add(a); model.add(b);
        SimTK_TEST_MUST_THROW_EXC(b->updInput("inL").connect(a->getOutput("out1")),
                                  OpenSim::Exception);
    }
    { // list output -> single-value input.
        TheWorld model;
        A* a = new A(); a->setName("a");
        B* b = new B(); b->setName("b");
        model.add(a); model.add(b);
        SimTK_TEST_MUST_THROW_EXC(
            b->updInput("in1").connect(a->getOutput("outL").getChannel("0")),
            OpenSim::Exception);
    }
    { // list output -> list input.
        TheWorld model;
        A* a = new A(); a->setName("a");
        B* b = new B(); b->setName("b");
        model.add(a); model.add(b);
        SimTK_TEST_MUST_THROW_EXC(
            b->updInput("inL").connect(a->getOutput("outL").getChannel("1")),
            OpenSim::Exception);
    }

    // Now check for exceptions when setting the connectee_name property, then
    // connecting on the model (similar to deserializing an XML model file).
    { // Connector.
        TheWorld model;
        B* b = new B(); b->setName("b");
        C* c = new C(); c->setName("c");
        model.add(b); model.add(c);
        c->updConnector("conn1").setConnecteeName("../b");
        SimTK_TEST_MUST_THROW_EXC(model.connect(), OpenSim::Exception);
    }
    { // single-value output -> single-value input.
        TheWorld model;
        A* a = new A(); a->setName("a");
        B* b = new B(); b->setName("b");
        model.add(a); model.add(b);
        b->updInput("in1").setConnecteeName("../a/out1");
        SimTK_TEST_MUST_THROW_EXC(model.connect(), OpenSim::Exception);
    }
    { // single-value output -> list input.
        TheWorld model;
        A* a = new A(); a->setName("a");
        B* b = new B(); b->setName("b");
        model.add(a); model.add(b);
        b->updInput("inL").appendConnecteeName("../a/out1");
        SimTK_TEST_MUST_THROW_EXC(model.connect(), OpenSim::Exception);
    }
    { // list output -> single-value input.
        TheWorld model;
        A* a = new A(); a->setName("a");
        B* b = new B(); b->setName("b");
        model.add(a); model.add(b);
        b->updInput("in1").setConnecteeName("../a/outL:2");
        SimTK_TEST_MUST_THROW_EXC(model.connect(), OpenSim::Exception);
    }
    { // list output -> list input.
        TheWorld model;
        A* a = new A(); a->setName("a");
        B* b = new B(); b->setName("b");
        model.add(a); model.add(b);
        b->updInput("inL").appendConnecteeName("../a/outL:0");
        SimTK_TEST_MUST_THROW_EXC(model.connect(), OpenSim::Exception);
    }
}


template<typename RowVec>
void assertEqual(const RowVec& a, const RowVec& b) {
    ASSERT(a.nrow() == b.nrow());
    ASSERT(a.ncol() == b.ncol());
    for(int i = 0; i < a.ncol(); ++i)
        ASSERT_EQUAL(a[i], b[i], 1e-10);
}

void testTableSource() {
    using namespace OpenSim;
    using namespace SimTK;

    {
        TheWorld model{};
        auto tablesource = new TableSourceVec3{};
        tablesource->setName("tablesource");
        tablesource->set_filename("testEformatParsing.trc");
        tablesource->set_tablename("markers");
        model.addComponent(tablesource);

        model.print("TestTableSource.osim");
    }

    {
    const std::string src_file{"TestTableSource.osim"};
    TheWorld model{src_file};
    const auto& tablesource = 
        model.getComponent<TableSourceVec3>("tablesource");
    model.print("TestTableSourceResult.osim");
    // Read the model file again to verify serialization.
    TheWorld model_copy{"TestTableSourceResult.osim"};
    const auto& tablesource_copy = 
        model_copy.getComponent<TableSourceVec3>("tablesource");
    OPENSIM_THROW_IF(tablesource_copy.get_filename() !=
                     tablesource.get_filename(),
                     OpenSim::Exception);
    }

    TimeSeriesTable table{};
    table.setColumnLabels({"0", "1", "2", "3"});
    SimTK::RowVector_<double> row{4, double{0}};
    for(unsigned i = 0; i < 4; ++i)
        table.appendRow(0.00 + 0.25 * i, row + i);

    std::cout << "Contents of the table :" << std::endl;
    std::cout << table << std::endl;

    auto tableSource = new TableSource{table};

    auto tableReporter = new TableReporter_<double, double>{};

    // Define the Simbody system
    MultibodySystem system;

    TheWorld theWorld;
    theWorld.setName("World");

    theWorld.add(tableSource);
    theWorld.add(tableReporter);

    tableReporter->updInput("inputs").connect(tableSource->getOutput("column"));

    theWorld.finalizeFromProperties();

    theWorld.connect();
    theWorld.buildUpSystem(system);

    const auto& report = tableReporter->getTable();

    State s = system.realizeTopology();

    s.setTime(0);
    tableReporter->report(s);
    assertEqual(table.getRowAtIndex(0)  , report.getRowAtIndex(0));

    s.setTime(0.1);
    tableReporter->report(s);
    row = RowVector_<double>{4, 0.4};
    assertEqual(row.getAsRowVectorView(), report.getRowAtIndex(1));

    s.setTime(0.25);
    tableReporter->report(s);
    assertEqual(table.getRowAtIndex(1)  , report.getRowAtIndex(2));

    s.setTime(0.4);
    tableReporter->report(s);
    row = RowVector_<double>{4, 1.6};
    assertEqual(row.getAsRowVectorView(), report.getRowAtIndex(3));

    s.setTime(0.5);
    tableReporter->report(s);
    assertEqual(table.getRowAtIndex(2)  , report.getRowAtIndex(4));

    s.setTime(0.6);
    tableReporter->report(s);
    row = RowVector_<double>{4, 2.4};
    assertEqual(row.getAsRowVectorView(), report.getRowAtIndex(5));

    s.setTime(0.75);
    tableReporter->report(s);
    assertEqual(table.getRowAtIndex(3)  , report.getRowAtIndex(6));

    std::cout << "Report: " << std::endl;
    std::cout << report << std::endl;
}


const std::string dataFileNameForInputConnecteeSerialization =
        "testComponentInterface_testInputConnecteeSerialization_data.sto";

void writeTimeSeriesTableForInputConnecteeSerialization() {
    TimeSeriesTable table{};
    table.setColumnLabels({"a", "b", "c", "d"});
    SimTK::RowVector row{4, 0.0}; row(1)=0.5; row(2)= 0.7; row(3)=0.8;
    for(unsigned i = 0; i < 4; ++i) table.appendRow(0.25 * i, row + i);
    STOFileAdapter_<double>::write(table,
                                   dataFileNameForInputConnecteeSerialization);
}

void testListInputConnecteeSerialization() {
    // We build a model, store the input connectee names, then
    // recreate the same model from a serialization, and make sure the
    // connectee names are the same.

    // Helper function.
    auto getConnecteeNames = [](const AbstractInput& in) {
        const auto numConnectees = in.getNumConnectees();
        std::vector<std::string> connecteeNames(numConnectees);
        for (int ic = 0; ic < numConnectees; ++ic) {
            connecteeNames[ic] = in.getConnecteeName(ic);
        }
        return connecteeNames;
    };

    // Build a model and serialize it.
    std::string modelFileName = "testComponentInterface_"
                                "testListInputConnecteeSerialization_world.xml";
    std::vector<std::string> expectedConnecteeNames{
            "../producer/column:a",
            "../producer/column:c",
            "../producer/column:b(berry)"};
    SimTK::Vector expectedInputValues;
    {
        // Create the "model," which just contains a reporter.
        TheWorld world;
        world.setName("World");
        
        // TableSource.
        auto* source = new TableSource();
        source->setName("producer");
        source->set_filename(dataFileNameForInputConnecteeSerialization);
        
        // TableReporter.
        auto* reporter = new TableReporter();
        reporter->setName("consumer");
        
        // Add to world.
        world.add(source);
        world.add(reporter);
        
        // Connect, finalize, etc.
        const auto& output = source->getOutput("column");
        // See if we preserve the ordering of the channels.
        reporter->updInput("inputs").connect(output.getChannel("a"));
        reporter->updInput("inputs").connect(output.getChannel("c"));
        // We want to make sure aliases are preserved.
        reporter->updInput("inputs").connect(output.getChannel("b"), "berry");
        world.finalizeFromProperties();
        world.connect();
        MultibodySystem system;
        world.buildUpSystem(system);
        
        // Grab the connectee names.
        const auto& input = reporter->getInput("inputs");
        SimTK_TEST(getConnecteeNames(input) == expectedConnecteeNames);
        
        // Get the value of the input at some given time.
        State s = system.realizeTopology();
        system.realize(s, Stage::Model);
        s.setTime(0.3);
        expectedInputValues = Input<double>::downcast(input).getVector(s);
        SimTK_TEST(expectedInputValues.size() == 3);
        
        // Serialize.
        world.print(modelFileName);
    }
    
    // Deserialize and test.
    {
        TheWorld world(modelFileName);
        const auto& reporter = world.getComponent("consumer");
        const auto& input = reporter.getInput("inputs");
        SimTK_TEST(input.isListConnector());
        // Check connectee names before *and* after connecting, since
        // the connecting process edits the connectee_name property.
        SimTK_TEST(getConnecteeNames(input) == expectedConnecteeNames);
        world.connect();
        SimTK_TEST(getConnecteeNames(input) == expectedConnecteeNames);
        // Check aliases.
        SimTK_TEST(input.getAlias(0) == ""); // default.
        SimTK_TEST(input.getAlias(1) == ""); // default.
        SimTK_TEST(input.getAlias(2) == "berry"); // specified.
        
        // Check that the value of the input is the same as before.
        MultibodySystem system;
        world.buildUpSystem(system);
        State s = system.realizeTopology();
        system.realize(s, Stage::Model);
        s.setTime(0.3);
        auto actualInputValues = Input<double>::downcast(input).getVector(s);
        
        SimTK_TEST_EQ(expectedInputValues, actualInputValues);
    }
}

void testSingleValueInputConnecteeSerialization() {

    // Test normal behavior of single-value input (de)serialization.
    // -------------------------------------------------------------
    
    // Build a model and serialize it.
    std::string modelFileName = "testComponentInterface_"
            "testSingleValueInputConnecteeSerialization_world.xml";
    double expectedInput1Value = SimTK::NaN;
    {
        // Create the "model," which just contains a reporter.
        TheWorld world;
        world.setName("World");
        
        // TableSource.
        auto* source = new TableSource();
        source->setName("producer");
        source->set_filename(dataFileNameForInputConnecteeSerialization);
        
        // TableReporter.
        auto* foo = new Foo();
        foo->setName("consumer");
        // Make sure we are dealing with single-value inputs
        // (future-proofing this test).
        SimTK_TEST(!foo->updInput("input1").isListConnector());
        SimTK_TEST(!foo->updInput("fiberLength").isListConnector());
        
        // Add to world.
        world.add(source);
        world.add(foo);
        
        // Connect, finalize, etc.
        const auto& output = source->getOutput("column");
        // See if we preserve the ordering of the channels.
        foo->updInput("input1").connect(output.getChannel("b"));
        // We want to make sure aliases are preserved.
        foo->updInput("fiberLength").connect(output.getChannel("d"), "desert");
        world.finalizeFromProperties();
        world.connect();
        MultibodySystem system;
        world.buildUpSystem(system);
        
        // Get the value of the input at some given time.
        State s = system.realizeTopology();
        system.realize(s, Stage::Model);
        s.setTime(0.3);
        const auto& input1 = foo->getInput("input1");
        expectedInput1Value = Input<double>::downcast(input1).getValue(s);
        
        // We won't wire up this input, but its connectee name should still
        // (de)serialize.
        foo->updInput("activation").setConnecteeName("non/existant");
        
        // Serialize.
        world.print(modelFileName);
    }
    
    // Deserialize and test.
    {
        TheWorld world(modelFileName);
        auto& foo = world.updComponent("consumer");
        const auto& input1 = foo.getInput("input1");
        const auto& fiberLength = foo.getInput("fiberLength");
        auto& activation = foo.updInput("activation");
        
        // Make sure these inputs are single-value after deserialization,
        // even before connecting.
        SimTK_TEST(!input1.isListConnector());
        SimTK_TEST(!fiberLength.isListConnector());
        SimTK_TEST(!activation.isListConnector());
        
        // Check connectee names before *and* after connecting, since
        // the connecting process edits the connectee_name property.
        SimTK_TEST(input1.getConnecteeName() == "../producer/column:b");
        SimTK_TEST(fiberLength.getConnecteeName() ==
                   "../producer/column:d(desert)");
        // Even if we hadn't wired this up, its name still deserializes:
        SimTK_TEST(activation.getConnecteeName() == "non/existant");
        // Now we must clear this before trying to connect, since the connectee
        // doesn't exist.
        activation.setConnecteeName("");
        
        // Connect.
        world.connect();
        
        // Make sure these inputs are single-value even after connecting.
        SimTK_TEST(!input1.isListConnector());
        SimTK_TEST(!fiberLength.isListConnector());
        SimTK_TEST(!activation.isListConnector());
        
        SimTK_TEST(input1.getConnecteeName() == "../producer/column:b");
        SimTK_TEST(fiberLength.getConnecteeName() ==
                   "../producer/column:d(desert)");
        
        // Check aliases.
        SimTK_TEST(input1.getAlias(0) == "");
        SimTK_TEST(fiberLength.getAlias(0) == "desert");
        
        // Check that the value of the input is the same as before.
        MultibodySystem system;
        world.buildUpSystem(system);
        State s = system.realizeTopology();
        system.realize(s, Stage::Model);
        s.setTime(0.3);
        
        SimTK_TEST_EQ(Input<double>::downcast(input1).getValue(s),
                      expectedInput1Value);
    }
    
    // Test error case: single-value input connectee_name has multiple values.
    // -----------------------------------------------------------------------
    // We'll first create an Input with multiple connectee_names (as is possible
    // in an XML file), then deserialize it and see what errors we get.
    std::string modelFileNameMultipleValues = "testComponentInterface_"
        "testSingleValueInputConnecteeSerializationMultipleValues_world.xml";
    {
        TheWorld world;
        auto* foo = new Foo();
        world.add(foo);
        
        // Hack into the Foo and modify its properties! The typical interface
        // for editing the input's connectee_name does not allow multiple
        // connectee names for a single-value input.
        auto& connectee_name = Property<ChannelPath>::updAs(
                        foo->updPropertyByName("input_input1_connectee_name"));
        connectee_name.setAllowableListSize(0, 10);
        connectee_name.appendValue(ChannelPath("apple"));
        connectee_name.appendValue(ChannelPath("banana"));
        connectee_name.appendValue(ChannelPath("lemon"));
        
        world.print(modelFileNameMultipleValues);
    }
    // Deserialize.
    {
        // Single-value connectee cannot have multiple connectee_names.
        // TODO Would ideally check for an exception, but we only emit a warning
        // for now. This is because the way we determine if multiple
        // connectee names were specified is by looking for spaces, but old
        // models used have spaces in their names.
        // SimTK_TEST_MUST_THROW_EXC(
        //     TheWorld world(modelFileNameMultipleValues),
        //     OpenSim::Exception);
        TheWorld world(modelFileNameMultipleValues);
    }
    
    // Test error case: connectee_name has invalid characters.
    // -------------------------------------------------------
    // This test is structured similarly to the one above.
    std::string modelFileNameInvalidChar = "testComponentInterface_"
        "testSingleValueInputConnecteeSerializationInvalidChar_world.xml";
    {
        TheWorld world;
        auto* foo = new Foo();
        world.add(foo);
        auto& input1 = foo->updInput("input1");
        // '+' is invalid for ComponentPath.
        SimTK_TEST_MUST_THROW_EXC(input1.setConnecteeName("abc+def"),
                                  OpenSim::Exception);
        // The check for invalid names occurs in
        // AbstractConnector::checkConnecteeNameProperty(), which is invoked
        // by the following function:
        // TODO how to still print file?
        // TODO foo->finalizeFromProperties(),
        // TODO                           OpenSim::Exception);
        
//        world.print(modelFileNameInvalidChar);
    }
    // Deserialize.
//    {
//        // Make sure that deserializing a Component with an invalid
//        // connectee_name throws an exception.
//        SimTK_TEST_MUST_THROW_EXC(TheWorld world(modelFileNameInvalidChar),
//                                  OpenSim::Exception);
//    }
}

void testAliasesAndLabels() {
    TheWorld* theWorld = new TheWorld();
    theWorld->setName("world");

    Foo* foo = new Foo();  foo->setName("foo");
    Foo* bar = new Foo();  bar->setName("bar");

    theWorld->addComponent(foo);
    theWorld->addComponent(bar);

    ASSERT_THROW(InputNotConnected, foo->getInput("input1").getAlias());
    ASSERT_THROW(InputNotConnected, foo->getInput("input1").getAlias(0));
    ASSERT_THROW(InputNotConnected, foo->updInput("input1").setAlias("qux"));
    ASSERT_THROW(InputNotConnected, foo->updInput("input1").setAlias(0, "qux"));
    ASSERT_THROW(InputNotConnected, foo->getInput("input1").getLabel());
    ASSERT_THROW(InputNotConnected, foo->getInput("input1").getLabel(0));

    // Non-list Input, no alias.
    foo->updInput("input1").connect( bar->getOutput("Output1") );
    SimTK_TEST(foo->getInput("input1").getAlias().empty());
    SimTK_TEST(foo->getInput("input1").getLabel() == "/world/bar/Output1");

    // Set alias.
    foo->updInput("input1").setAlias("waldo");
    SimTK_TEST(foo->getInput("input1").getAlias() == "waldo");
    SimTK_TEST(foo->getInput("input1").getLabel() == "waldo");

    foo->updInput("input1").setAlias(0, "fred");
    SimTK_TEST(foo->getInput("input1").getAlias() == "fred");
    SimTK_TEST(foo->getInput("input1").getLabel() == "fred");

    using SimTKIndexOutOfRange = SimTK::Exception::IndexOutOfRange;
    ASSERT_THROW(SimTKIndexOutOfRange, foo->getInput("input1").getAlias(1));
    ASSERT_THROW(SimTKIndexOutOfRange, foo->updInput("input1").setAlias(1, "fred"));
    ASSERT_THROW(SimTKIndexOutOfRange, foo->getInput("input1").getLabel(1));

    foo->updInput("input1").disconnect();

    // Non-list Input, with alias.
    foo->updInput("input1").connect( bar->getOutput("Output1"), "baz" );
    SimTK_TEST(foo->getInput("input1").getAlias() == "baz");
    SimTK_TEST(foo->getInput("input1").getLabel() == "baz");

    // List Input, no aliases.
    foo->updInput("listInput1").connect( bar->getOutput("Output1") );
    foo->updInput("listInput1").connect( bar->getOutput("Output3") );

    ASSERT_THROW(OpenSim::Exception, foo->getInput("listInput1").getAlias());
    ASSERT_THROW(OpenSim::Exception, foo->getInput("listInput1").getLabel());

    SimTK_TEST(foo->getInput("listInput1").getAlias(0).empty());
    SimTK_TEST(foo->getInput("listInput1").getLabel(0) == "/world/bar/Output1");

    SimTK_TEST(foo->getInput("listInput1").getAlias(1).empty());
    SimTK_TEST(foo->getInput("listInput1").getLabel(1) == "/world/bar/Output3");

    foo->updInput("listInput1").disconnect();

    // List Input, with aliases.
    foo->updInput("listInput1").connect( bar->getOutput("Output1"), "plugh" );
    foo->updInput("listInput1").connect( bar->getOutput("Output3"), "thud" );

    SimTK_TEST(foo->getInput("listInput1").getAlias(0) == "plugh");
    SimTK_TEST(foo->getInput("listInput1").getLabel(0) == "plugh");

    SimTK_TEST(foo->getInput("listInput1").getAlias(1) == "thud");
    SimTK_TEST(foo->getInput("listInput1").getLabel(1) == "thud");
}

int main() {

    //Register new types for testing deserialization
    Object::registerType(Foo());
    Object::registerType(Bar());
    Object::registerType(TheWorld());

    SimTK_START_TEST("testComponentInterface");
        SimTK_SUBTEST(testMisc);
        SimTK_SUBTEST(testListInputs);
        SimTK_SUBTEST(testListConnectors);
        SimTK_SUBTEST(testComponentPathNames);
        SimTK_SUBTEST(testInputOutputConnections);
        SimTK_SUBTEST(testInputConnecteeNames);
        SimTK_SUBTEST(testExceptionsForConnecteeTypeMismatch);
        SimTK_SUBTEST(testTableSource);
        SimTK_SUBTEST(testAliasesAndLabels);
    
        writeTimeSeriesTableForInputConnecteeSerialization();
        SimTK_SUBTEST(testListInputConnecteeSerialization);
        SimTK_SUBTEST(testSingleValueInputConnecteeSerialization);
    SimTK_END_TEST();
}
