/* -------------------------------------------------------------------------- *
 *                OpenSim:  testComponentInterface.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Common/TableSource.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/CommonUtilities.h>
#include <simbody/internal/SimbodyMatterSubsystem.h>
#include <simbody/internal/GeneralForceSubsystem.h>
#include <simbody/internal/Force.h>
#include <simbody/internal/MobilizedBody_Pin.h>
#include <simbody/internal/MobilizedBody_Ground.h>

#include <catch2/catch_all.hpp>
#include <random>

namespace
{
    // compiler test (see issue #3468)
    //
    // - this component must be compiled before the `using namespace` declarations below
    // - this component must be compile-able
    // - it exists to test whether OpenSim's component+macro system works for components
    //   defined in their own namespace, without an implict dependency on `using namespace OpenSim`
    // - this is effectively a compiler assertion that the OpenSim macros work as expected
    // - more information available in issue #3468
    class ComponentInAnonymousNamespace : public OpenSim::Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(ComponentInAnonymousNamespace, OpenSim::Component);

        double getDoubleOutput(const SimTK::State&) const { return 0.0; }
    public:
        OpenSim_DECLARE_PROPERTY(some_prop, SimTK::Vec3, "comment");
        OpenSim_DECLARE_INPUT(some_input, float, SimTK::Stage::Acceleration, "some other comment");
        OpenSim_DECLARE_OUTPUT(some_output, double, getDoubleOutput, SimTK::Stage::Model);
        OpenSim_DECLARE_SOCKET(some_socket, OpenSim::Component, "some socket comment");
    };

    class ComponentWithOptionalSimpleProperty final : public OpenSim::Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(ComponentWithOptionalSimpleProperty, OpenSim::Component);
    public:
        OpenSim_DECLARE_OPTIONAL_PROPERTY(num, int, "");

        ComponentWithOptionalSimpleProperty()
        {
            constructProperty_num();
        }
    };

    class ComponentWithOptionalObjectProperty final : public OpenSim::Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(ComponentWithOptionalObjectProperty, OpenSim::Component);
    public:
        OpenSim_DECLARE_OPTIONAL_PROPERTY(func, OpenSim::Function, "");

        ComponentWithOptionalObjectProperty()
        {
            constructProperty_func();
        }
    };

    class BlankComponent final : public OpenSim::Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(BlankComponent, OpenSim::Component);
    };

    class ComponentWithObjectProperty final : public OpenSim::Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(ComponentWithObjectProperty, OpenSim::Component);
    public:
        OpenSim_DECLARE_PROPERTY(object_property, BlankComponent, "an object property");

        ComponentWithObjectProperty() { constructProperty_object_property(BlankComponent{}); }
    };
}

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
        addDiscreteVariable("dvX", Stage::Dynamics);
        addModelingOption("moX", 2);
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
        Super::finalizeConnections(*this);
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
    void extendFinalizeConnections(Component& root) override {
        Super::extendFinalizeConnections(root);
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

    OpenSim_DECLARE_SOCKET(parentFoo, Foo, "");
    OpenSim_DECLARE_SOCKET(childFoo, Foo, "");
    OpenSim_DECLARE_LIST_SOCKET(listFoo, Foo, "");

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
        const SimTK::Force& force = forces.getForce(fix);
        const auto& spring = SimTK::Force::TwoPointLinearSpring::downcast(force);

        return spring.calcPotentialEnergyContribution(state);
    }

    /** Returns the `this` pointer. Used to ensure that the std::function
     within Outputs is properly copied when copying components. */
    size_t myself(const SimTK::State& s) const { return size_t(this); }

    double getCopytestingMemVar(const SimTK::State& s) const
    { return copytestingViaMemberVariable; }

    class ParentAndFooAreSame : OpenSim::Exception {
    public:
        using OpenSim::Exception::Exception;
    };
protected:
    /** Component Interface */
    void extendFinalizeConnections(Component& root) override{
        Super::extendFinalizeConnections(root);
        // do any internal wiring
        world = dynamic_cast<TheWorld*>(&root);
        // perform custom checking
        if (&updSocket<Foo>("parentFoo").getConnectee()
                == &updSocket<Foo>("childFoo").getConnectee()){
            string msg = "ERROR - Bar::extendFinalizeConnections()\n";
            msg += " parentFoo and childFoo cannot be the same component.";
            throw ParentAndFooAreSame(msg);
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

            SimTK::Force::TwoPointLinearSpring
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

        // Add a modeling option (mo) and a discrete variable (dv) as though
        // they were allocated natively in Simbody.
        // This will allow testing the ability to accomodate a mo and dv that
        // were allocated from subsystem different than the default subsystem
        // and to access a dv that is not type double.
        // The following calls put the mo and dv into the maps used to contain
        // all mo's and dv's exposed in OpenSim. When Stage::Topology is
        // realized, they will be allocated in class Bar's override of
        // extendRealizeTopology(). See below.
        bool allocate = false;
        int maxFlagValue = 1;
        addDiscreteVariable("point", Stage::Position, allocate);
        addModelingOption("moY", maxFlagValue, allocate);
    }

    // Manually allocate and update the index and subsystem for
    // a discrete variable and a modeling option as though they were
    // natively allocated in Simbody and brought into OpenSim.
    // Note, as of May 2024, this is also what one would need to do in order
    // to add a discrete variable that is a type other than double.
    void extendRealizeTopology(SimTK::State& state) const override {
        Super::extendRealizeTopology(state);

        GeneralForceSubsystem& fsub = world->updForceSubsystem();

        // Discrete Variable Initialization
        std::string dvName = "point";
        Vec3 point(0.0, 0.1, 0.2);
        SimTK::DiscreteVariableIndex dvIndex =
            fsub.allocateDiscreteVariable(state, Stage::Dynamics,
                new Value<Vec3>(point));
        initializeDiscreteVariableIndexes(dvName,
            fsub.getMySubsystemIndex(), dvIndex);

        // Check the discrete variable indexes
        SimTK::SubsystemIndex ssIndexTestDV;
        SimTK::DiscreteVariableIndex dvIndexTest;
        CHECK_THROWS_AS(
            getDiscreteVariableIndexes("typo", ssIndexTestDV, dvIndexTest),
            VariableNotFound);
        double value;
        getDiscreteVariableIndexes(dvName, ssIndexTestDV, dvIndexTest);
        REQUIRE(ssIndexTestDV == fsub.getMySubsystemIndex());
        REQUIRE(dvIndexTest == dvIndex);

        // Modeling Option Initialization
        std::string moName = "moY";
        int moVal{0};
        SimTK::DiscreteVariableIndex moIndex =
            fsub.allocateDiscreteVariable(state, Stage::Dynamics,
                new Value<int>(moVal));
        initializeModelingOptionIndexes(moName,
            fsub.getMySubsystemIndex(), moIndex);

        // Check the modeling option indexes
        SimTK::SubsystemIndex ssIndexTestMO;
        SimTK::DiscreteVariableIndex moIndexTest;
        getModelingOptionIndexes(moName, ssIndexTestMO, moIndexTest);
        CHECK_THROWS_AS(
            getModelingOptionIndexes("typo", ssIndexTestMO, moIndexTest),
            VariableNotFound);
        REQUIRE(ssIndexTestMO == fsub.getMySubsystemIndex());
        REQUIRE(moIndexTest == moIndex);
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
    OpenSim_DECLARE_PROPERTY(Foo3, Foo, "3rd Foo of CompoundFoo");
    OpenSim_DECLARE_PROPERTY(scale1, double, "Scale factor for 1st Foo");
    OpenSim_DECLARE_PROPERTY(scale2, double, "Scale factor for 2nd Foo");
    OpenSim_DECLARE_PROPERTY(scale3, double, "Scale factor for 3rd Foo");

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
        Foo& foo3 = upd_Foo3();

        // update CompoundFoo's properties based on it sub Foos
        double orig_mass = get_mass();
        upd_mass() = get_scale1()*foo1.get_mass() + get_scale2()*foo2.get_mass()
            + get_scale3()*foo3.get_mass();

        double inertiaScale = (get_mass() / orig_mass);

        for (int i = 0; i < updProperty_inertia().size(); ++i) {
            upd_inertia(i) = inertiaScale*get_inertia(i);
        }
    }

private:
    void constructProperties() {
        constructProperty_Foo1(Foo());
        constructProperty_Foo2(Foo());
        constructProperty_Foo3(Foo());
        constructProperty_scale1(1.0);
        constructProperty_scale2(2.0);
        constructProperty_scale3(3.0);
    }
}; // End of Class CompoundFoo

SimTK_NICETYPENAME_LITERAL(Foo);
SimTK_NICETYPENAME_LITERAL(Bar);

TEST_CASE("Component Interface Misc.")
{
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

        ASSERT_THROW(ComponentHasNoSystem,
                theWorld.traverseToStateVariable(varName));
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

    //Configure the socket to look for its dependency by this name
    //Will get resolved and connected automatically at Component connect
    bar.updSocket<Foo>("parentFoo").setConnecteePath(
            foo.getAbsolutePathString());
    bar.connectSocket_childFoo(foo);

    // add a subcomponent
    // connect internals
    ASSERT_THROW( Bar::ParentAndFooAreSame,
                  theWorld.connect() );


    auto worldTreeAsList = theWorld.getComponentList();
    std::cout << "list begin: " << worldTreeAsList.begin()->getName() << std::endl;
    for (auto it = worldTreeAsList.begin();
              it != worldTreeAsList.end(); ++it) {
        std::cout << "Iterator is at: " << it->getAbsolutePathString() << std::endl;
    }


    std::cout << "Using range-for loop: " << std::endl;
    for (const Component& component : worldTreeAsList) {
        std::cout << "Iterator is at: " << component.getAbsolutePathString() << std::endl;
    }


    std::cout << "Iterate over only Foo's." << std::endl;
    for (auto& component : theWorld.getComponentList<Foo>()) {
        std::cout << "Iterator is at: " << component.getAbsolutePathString() << std::endl;
    }

    Foo& foo2 = *new Foo();
    foo2.setName("Foo2");
    foo2.set_mass(3.0);

    theWorld.add(&foo2);

    // Add more Foos to connect to Bar's list socket. This will satisfy the
    // model's connections and let us print it below.
    Foo& foo3 = *new Foo();
    foo3.setName("Foo3");
    theWorld.add(&foo3);
    foo3.set_mass(4.0);
    bar.appendSocketConnectee_listFoo(foo3);

    Foo& foo4 = *new Foo();
    foo4.setName("Foo4");
    theWorld.add(&foo4);
    foo4.set_mass(5.0);
    bar.appendSocketConnectee_listFoo(foo4);

    std::cout << "Iterate over Foo's after adding Foo2, Foo3, and Foo4." << std::endl;
    for (auto& component : theWorld.getComponentList<Foo>()) {
        std::cout << "Iter at: " << component.getAbsolutePathString() << std::endl;
    }

    // Query existing components.
    theWorld.printComponentsMatching("");
    SimTK_TEST(theWorld.hasComponent("Foo"));
    SimTK_TEST(!theWorld.hasComponent("Nonexistent"));
    SimTK_TEST(theWorld.hasComponent<Foo>("Foo"));
    SimTK_TEST(!theWorld.hasComponent<Bar>("Foo"));
    SimTK_TEST(!theWorld.hasComponent<Foo>("Nonexistent"));

    bar.connectSocket_childFoo(foo2);
    string socketName = bar.updSocket<Foo>("childFoo").getName();

    // do any other input/output connections
    foo.connectInput_input1(bar.getOutput("PotentialEnergy"));

    // Bar should connect now
    theWorld.connect();
    theWorld.buildUpSystem(system);

    const Foo& foo2found = theWorld.getComponent<Foo>("Foo2");
    ASSERT(foo2 == foo2found);

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

    world2->updComponent("Bar").getSocket<Foo>("childFoo");

    // in older versions of OpenSim, calling `getConnectee` on a socket
    // that hasn't been `connect`ed or `finalizeConnection`ed yet was
    // an exception.
    //
    // newer versions fall back to a (slower) runtime lookup of the connectee
    // using the connectee's component path
    // SimTK_TEST_MUST_THROW_EXC(
    //     world2->updComponent("Bar").getConnectee<Foo>("childFoo"),
    //     OpenSim::Exception
    // );

    ASSERT(theWorld == *world2, __FILE__, __LINE__,
        "Model serialization->deserialization FAILED");

    world2->setName("InternalWorld");
    world2->connect();

    world2->updComponent("Bar").getSocket<Foo>("childFoo");
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

    world3.getComponent("Bar").getSocket<Foo>("parentFoo");

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
    compFoo.set_Foo3(foo3);
    compFoo.finalizeFromProperties();

    world3.add(&compFoo);
    world3.add(&bar2);

    //Configure the socket to look for its dependency by this name
    //Will get resolved and connected automatically at Component connect
    bar2.updSocket<Foo>("parentFoo").setConnecteePath(
            compFoo.getRelativePathString(bar2));

    bar2.connectSocket_childFoo(compFoo.get_Foo1());
    bar2.appendSocketConnectee_listFoo(compFoo.get_Foo3());
    compFoo.upd_Foo1().updInput("input1")
        .connect(bar2.getOutput("PotentialEnergy"));

    world3.connect();
    world3.print("Compound_" + modelFile);

    cout << "Adding world3 to theWorld" << endl;
    theWorld.add(world3.clone());

    // Should not be able to add the same Component twice within the same tree
    ASSERT_THROW( ComponentAlreadyPartOfOwnershipTree,
                  world3.add(&bar2));

    cout << "Connecting theWorld:" << endl;
    //theWorld.dumpSubcomponents();
    theWorld.printSubcomponentInfo();
    theWorld.printOutputInfo();
    theWorld.finalizeFromProperties();
    theWorld.connect();

    auto* reporter = new TableReporterVector();
    reporter->set_report_time_interval(0.1);
    reporter->connectInput_inputs(foo.getOutput("Qs"));
    theWorld.add(reporter);

    // Connect our state variables.
    foo.connectInput_fiberLength(bar.getOutput("fiberLength"));
    foo.connectInput_activation(bar.getOutput("activation"));

    MultibodySystem system3;
    cout << "Building theWorld's system:" << endl;
    theWorld.buildUpSystem(system3);

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

    theWorld.printSubcomponentInfo();
    theWorld.printOutputInfo();

    std::cout << "Iterate over all Components in the world." << std::endl;
    for (auto& component : theWorld.getComponentList<Component>()) {
        std::cout << "Iterator is at: " << component.getAbsolutePathString() << std::endl;
    }

    // Should fail to get Component when path is not specified
    ASSERT_THROW(OpenSim::Exception,
        theWorld.getComponent<CompoundFoo>("BigFoo") );

    // With path to the component it should work
    auto& bigFoo = theWorld.getComponent<CompoundFoo>("World3/BigFoo");
    // const Sub& topSub = theWorld.getComponent<Sub>("InternalWorld/internalSub");

    // Should also be able to get top-level
    auto& topFoo = theWorld.getComponent<Foo>("Foo2");
    cout << "Top level Foo2 path name: " << topFoo.getAbsolutePathString() << endl;

    // And the leaf Foo2 from BigFoo
    auto& leafFoo = bigFoo.getComponent<Foo>("Foo2");
    cout << "Leaf level Foo2 path name: " << leafFoo.getAbsolutePathString() << endl;

    theWorld.print("Nested_" + modelFile);
}

// un-disable test for duplicate names when we re-enable the exception
TEST_CASE("Component Interface Throws on Duplicate Names", "[.disabled]")
{
    TheWorld theWorld;
    theWorld.setName("World");
    theWorld.finalizeFromProperties();

    Foo* a = new Foo();
    a->setName("A");

    Foo* b = new Foo();
    b->setName("B");

    theWorld.addComponent(a);
    theWorld.addComponent(b);

    b->setName("A");

    SimTK_TEST_MUST_THROW_EXC(
        theWorld.finalizeFromProperties(),
        OpenSim::SubcomponentsWithDuplicateName );

}

// In order to access subcomponents in a copy, One must invoke
// finalizeFromProperties() after copying. This test makes sure that you get an
// exception if you did not call finalizeFromProperties() before calling a
// method like getComponentList().
TEST_CASE("Component Interface Exceptions Finalize From Properties After Copy")
{
    TheWorld theWorld;
    {
        MultibodySystem system;
        Foo* foo = new Foo();
        theWorld.add(foo);
    }
    {
        TheWorld copy = theWorld;
        SimTK_TEST_MUST_THROW_EXC(copy.getComponentList(), ComponentIsAnOrphan);
    }
}

TEST_CASE("Component Interface List Inputs")
{
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

    bar.connectSocket_parentFoo(foo);
    bar.connectSocket_childFoo(foo2);

    auto* reporter = new ConsoleReporter();
    reporter->setName("rep0");
    theWorld.add(reporter);

    // wire up console reporter inputs to desired model outputs
    reporter->connectInput_inputs(foo.getOutput("Output1"));
    reporter->connectInput_inputs(bar.getOutput("PotentialEnergy"));
    reporter->connectInput_inputs(bar.getOutput("fiberLength"));
    reporter->connectInput_inputs(bar.getOutput("activation"));

    auto* tabReporter = new TableReporter();
    tabReporter->setName("TableReporterMixedOutputs");
    theWorld.add(tabReporter);

    // wire up table reporter inputs (using convenience method) to desired
    // model outputs
    tabReporter->addToReport(bar.getOutput("fiberLength"));
    tabReporter->addToReport(bar.getOutput("activation"));
    tabReporter->addToReport(foo.getOutput("Output1"));
    tabReporter->addToReport(bar.getOutput("PotentialEnergy"));

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

TEST_CASE("Component Interface Sockets")
{
    MultibodySystem system;
    TheWorld theWorld;
    theWorld.setName("world");

    Foo& foo1 = *new Foo(); foo1.setName("foo1"); foo1.set_mass(2.0);
    theWorld.add(&foo1);

    Foo& foo2 = *new Foo(); foo2.setName("foo2"); foo2.set_mass(3.0);
    theWorld.add(&foo2);

    Foo& foo3 = *new Foo(); foo3.setName("foo3"); foo3.set_mass(4.0);
    theWorld.add(&foo3);

    Foo& foo4 = *new Foo(); foo4.setName("foo4"); foo4.set_mass(5.0);
    theWorld.add(&foo4);

    Foo& foo5 = *new Foo(); foo5.setName("foo5"); foo5.set_mass(6.0);
    theWorld.add(&foo5);

    Bar& bar = *new Bar(); bar.setName("bar");
    theWorld.add(&bar);

    SECTION("Sockets not connected") {
        CHECK_THROWS(theWorld.connect());
    }

    // Connect the single-object sockets.
    bar.connectSocket_parentFoo(foo1);
    bar.connectSocket_childFoo(foo2);

    SECTION("Single-object Sockets") {
        // Ensure that calling connect() on bar's "parentFoo" doesn't increase
        // its number of connectees.
        bar.connectSocket_parentFoo(foo1);
        CHECK(bar.getSocket<Foo>("parentFoo").getNumConnectees() == 1);

        theWorld.connect();
        theWorld.buildUpSystem(system);
        State s = system.realizeTopology();
        CHECK(bar.getConnectee<Foo>("parentFoo").get_mass() == 2.0);
    }

    SECTION("List Sockets") {
        // Check that the list socket is empty, since we haven't made any
        // connections yet.
        theWorld.connect();
        CHECK(bar.getSocket<Foo>("listFoo").getNumConnectees() == 0);

        // Connect to list socket.
        bar.appendSocketConnectee_listFoo(foo1);
        bar.appendSocketConnectee_listFoo(foo2);
        theWorld.connect();
        theWorld.buildUpSystem(system);
        CHECK(bar.getSocket<Foo>("listFoo").getNumConnectees() == 2);

        // Check that connecting to the same component throws an Exception.
        CHECK_THROWS_WITH(bar.appendSocketConnectee_listFoo(foo2),
                Catch::Matchers::ContainsSubstring("Socket 'listFoo' already "
                    "has a connectee of type 'Foo' named 'foo2'."));

        bar.appendSocketConnectee_listFoo(foo3);
        theWorld.connect();
        theWorld.buildUpSystem(system);
        CHECK(bar.getSocket<Foo>("listFoo").getNumConnectees() == 3);
    }
}

TEST_CASE("Component Interface tryGetSocket")
{
    Bar bar;
    REQUIRE(bar.tryGetSocket("parentFoo") != nullptr);
    REQUIRE(bar.tryGetSocket("childFoo") != nullptr);
    REQUIRE(bar.tryGetSocket("NonExistentFoo") == nullptr);
}

TEST_CASE("Component Interface getSocket")
{
    Bar bar;
    REQUIRE_NOTHROW(bar.getSocket("parentFoo"));
    REQUIRE_NOTHROW(bar.getSocket("childFoo"));
    REQUIRE_THROWS_AS(bar.getSocket("NonExistentFoo"), SocketNotFound);
}

TEST_CASE("Component Interface tryUpdSocket")
{
    Bar bar;
    REQUIRE(bar.tryUpdSocket("parentFoo") != nullptr);
    REQUIRE(bar.tryUpdSocket("childFoo") != nullptr);
    REQUIRE(bar.tryUpdSocket("NonExistentFoo") == nullptr);
}

TEST_CASE("Component Interface updSocket")
{
    Bar bar;
    REQUIRE_NOTHROW(bar.updSocket("parentFoo"));
    REQUIRE_NOTHROW(bar.updSocket("childFoo"));
    REQUIRE_THROWS_AS(bar.updSocket("NonExistentFoo"), SocketNotFound);
}

TEST_CASE("Component Interface tryGetOutput")
{
    Foo foo;
    REQUIRE(foo.tryGetOutput("Output1") != nullptr);
    REQUIRE(foo.tryGetOutput("NonExistentOutput") == nullptr);
}

TEST_CASE("Component Interface tryUpdOutput")
{
    Foo foo;
    REQUIRE(foo.tryUpdOutput("Output1") != nullptr);
    REQUIRE(foo.tryUpdOutput("NonExistentOutput") == nullptr);
}

TEST_CASE("Component Interface Socket::canConnectTo")
{
    TheWorld theWorld;
    theWorld.setName("world");

    Foo& foo = *new Foo(); foo.setName("foo"); foo.set_mass(2.0);
    theWorld.add(&foo);

    Foo& foo2 = *new Foo(); foo2.setName("foo2"); foo2.set_mass(3.0);
    theWorld.add(&foo2);

    Bar& bar = *new Bar(); bar.setName("bar");
    theWorld.add(&bar);

    SimTK_TEST(bar.getSocket("parentFoo").canConnectTo(foo));
    SimTK_TEST(bar.getSocket("childFoo").canConnectTo(foo2));

    // `theWorld` is an invalid type for the socket, so `canConnectTo` should
    // return `false`
    SimTK_TEST(!bar.getSocket("parentFoo").canConnectTo(theWorld));
    SimTK_TEST(!bar.getSocket("childFoo").canConnectTo(theWorld));
}

TEST_CASE("Component Interface Input::canConnectTo")
{
    TheWorld theWorld;
    theWorld.setName("world");

    Foo& foo = *new Foo(); foo.setName("foo"); foo.set_mass(2.0);
    theWorld.add(&foo);

    Foo& foo2 = *new Foo(); foo2.setName("foo2"); foo2.set_mass(3.0);
    theWorld.add(&foo2);

    Bar& bar = *new Bar(); bar.setName("bar");
    theWorld.add(&bar);

    // an arbitrary object cannot be connected to an input (the
    // argument must at least be an AbstractOutput), so these
    // should return `false`, always.
    SimTK_TEST(!foo.getInput("input1").canConnectTo(theWorld));
    SimTK_TEST(!foo.getInput("input1").canConnectTo(foo));
    SimTK_TEST(!foo.getInput("input1").canConnectTo(foo2));
    SimTK_TEST(!foo.getInput("input1").canConnectTo(bar));
}

TEST_CASE("Component Interface Component Path Names")
{
    Foo foo;
    foo.setName("LegWithConstrainedFoot/foot");
    // verify that this illegal name throws when we try to finalize
    // the component from its property values
    ASSERT_THROW(InvalidComponentName, foo.finalizeFromProperties());

    // Build using real components and assemble them
    // into a tree and test the path names that are
    // generated on the fly.
    TheWorld top;
    TheWorld otherTop;

    top.setName("Top");
    otherTop.setName("OtherTop");

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

    top.printSubcomponentInfo();
    top.printOutputInfo();

    std::string absPathC = C->getAbsolutePathString();
    ASSERT(absPathC == "/A/B/C");

    std::string absPathE = E->getAbsolutePathString();
    ASSERT(absPathE == "/A/D/E");

    // Specific tests to relative path name facilities
    std::string EWrtB = E->getRelativePathString(*B);
    ASSERT(EWrtB == "../D/E"); // "/A/B/" as common

    std::string BWrtE = B->getRelativePathString(*E);
    ASSERT(BWrtE == "../../B"); // "/A/" as common

    // null case component wrt itself
    std::string fooWrtFoo = D->getRelativePathString(*D);
    ASSERT(fooWrtFoo == "");

    std::string CWrtOtherTop = C->getRelativePathString(otherTop);
    ASSERT(CWrtOtherTop == "A/B/C");

    std::string OtherTopWrtC = otherTop.getRelativePathString(*C);
    ASSERT(OtherTopWrtC == "../../..");

    // Must specify a unique path to E
    ASSERT_THROW(OpenSim::ComponentNotFoundOnSpecifiedPath,
                 /*auto& eref = */top.getComponent("E") );

    auto& cref = top.getComponent(absPathC);
    auto& eref = top.getComponent(absPathE);

    auto cFromE = cref.getRelativePathString(eref);
    ASSERT(cFromE == "../../B/C");

    auto eFromC = eref.getRelativePathString(cref);
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

    top.printSubcomponentInfo();
    top.printOutputInfo();

    std::string fFoo1AbsPath =
        F->getComponent<Foo>("Foo1").getAbsolutePathString();
    std::string aBar2AbsPath =
        A->getComponent<Bar>("Bar2").getAbsolutePathString();
    auto bar2FromBarFoo =
        bar2->getRelativePathString(F->getComponent<Foo>("Foo1"));

    // Verify deep copy of subcomponents
    const Foo& foo1inA = top.getComponent<Foo>("/A/Foo1");
    const Foo& foo1inF = top.getComponent<Foo>("/F/Foo1");
    ASSERT(&foo1inA != &foo1inF);

    // double check that we have the original Foo foo1 in A
    ASSERT(&foo1inA == foo1);

    // This bar2 that belongs to A and connects the two foo2s
    bar2->connectSocket_parentFoo(*foo2);
    bar2->connectSocket_childFoo(F->getComponent<Foo>("Foo2"));

    // auto& foo2inF = bar2->getComponent<Foo>("../../F/Foo2");

    // now wire up bar2 that belongs to F and connect the
    // two foo1s one in A and other F
    auto& fbar2 = F->updComponent<Bar>("Bar2");
    ASSERT(&fbar2 != bar2);

    fbar2.connectSocket_parentFoo(*foo1);
    fbar2.updSocket<Foo>("childFoo")
         .setConnecteePath("../Foo1");

    top.printSubcomponentInfo();
    top.printOutputInfo();
    top.connect();
}

TEST_CASE("Component Interface Component::findComponent")
{
    class A : public Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(A, Component);
    public:
        A(const std::string& name) { setName(name); }
    };
    class B : public Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(B, Component);
    public:
        OpenSim_DECLARE_SOCKET(socket_a, A, "");
        B(const std::string& name) { setName(name); }
    };

    // Build a model.
    A top("top");
    B* b1 = new B("b1");
    b1->connectSocket_socket_a(top);
    top.addComponent(b1);
    B* b2 = new B("b2");
    b1->addComponent(b2);
    B* b3 = new B("b3");
    b2->addComponent(b3);

    // Test findComponent().
    B* duplicate1 = new B("duplicate");
    b1->addComponent(duplicate1);
    B* duplicate2 = new B("duplicate");
    b2->addComponent(duplicate2);
    SimTK_TEST(top.findComponent("nonexistant") == nullptr);
    SimTK_TEST(top.findComponent("b1") == b1);
    SimTK_TEST(top.findComponent<B>("b1") == b1);
    SimTK_TEST(b3->findComponent("b1") == nullptr);
    SimTK_TEST(top.findComponent<A>("b1") == nullptr);

    SimTK_TEST_MUST_THROW_EXC(top.findComponent("duplicate"),
            OpenSim::Exception);

    // Test AbstractSocket::findAndConnect().
    b3->updSocket("socket_a").findAndConnect("top");
    SimTK_TEST(&b3->getConnectee<A>("socket_a") == &top);
    // Connectee has the wrong type.
    SimTK_TEST_MUST_THROW_EXC(b3->updSocket("socket_a").findAndConnect("b1"),
            OpenSim::Exception);

    // Test partial paths.
    A* a3 = new A("a3");
    b2->addComponent(a3);
    b3->updSocket("socket_a").findAndConnect("b2/a3");
    SimTK_TEST(&b3->getConnectee<A>("socket_a") == a3);

    // This works, even though a3 is not in b3.
    b3->updSocket("socket_a").findAndConnect("b3/a3");
    SimTK_TEST(&b3->getConnectee<A>("socket_a") == a3);
}

TEST_CASE("Component Interface can Traverse Path to Component")
{
    class A : public Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(A, Component);
    public:
        A(const std::string& name) { setName(name); }
    };
    class B : public Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(B, Component);
    public:
        B(const std::string& name) { setName(name); }
    };

    // Add lots of subcomponents to check the performance of
    // traversePathToSubcomponent().
    auto addLotsOfSubcomponents = [](Component& c) {
        for (int i = 0; i < 100; ++i) {
            c.addComponent(new A("unuseda" + std::to_string(i)));
            c.addComponent(new B("unusedb" + std::to_string(i)));
        }
    };

    A top("top");
    addLotsOfSubcomponents(top);

    A* a1 = new A("a1");
    addLotsOfSubcomponents(*a1);
    top.addComponent(a1);
    B* b1 = new B("b1");
    addLotsOfSubcomponents(*b1);
    top.addComponent(b1);

    A* a2 = new A("a2");
    addLotsOfSubcomponents(*a2);
    a1->addComponent(a2);
    B* b2 = new B("b2");
    addLotsOfSubcomponents(*b2);
    a1->addComponent(b2);

    // top.printSubcomponentInfo();

    // Self.
    SimTK_TEST(&top.getComponent<A>("") == &top);
    SimTK_TEST(&top.getComponent<A>(".") == &top);
    SimTK_TEST(&a1->getComponent<A>("") == a1);
    SimTK_TEST(&b2->getComponent<B>("") == b2);

    SimTK_TEST(&top.getComponent<A>("a1") == a1);
    SimTK_TEST(&top.getComponent<A>("a1/") == a1);
    SimTK_TEST(&top.getComponent<B>("b1") == b1);
    SimTK_TEST(&top.getComponent<A>("a1/a2") == a2);
    SimTK_TEST(&top.getComponent<B>("a1/b2") == b2);
    // Going up.
    SimTK_TEST(&a1->getComponent<A>("..") == &top);
    SimTK_TEST(&a2->getComponent<A>("../../") == &top);
    SimTK_TEST(&b2->getComponent<A>("../../") == &top);
    SimTK_TEST(&a2->getComponent<A>("..") == a1);
    SimTK_TEST(&b2->getComponent<A>("..") == a1);
    // Going up and then back down.
    SimTK_TEST(&a1->getComponent<A>("../a1") == a1);
    SimTK_TEST(&a1->getComponent<B>("../b1") == b1);
    SimTK_TEST(&a1->getComponent<B>("../a1/b2") == b2);
    // Absolute paths.
    SimTK_TEST(&top.getComponent<A>("/a1") == a1);
    SimTK_TEST(&b1->getComponent<B>("/a1/b2") == b2);
    SimTK_TEST(&b2->getComponent<A>("/a1/a2") == a2);
    SimTK_TEST(&top.getComponent<A>("/") == &top);


    // No component.
    // -------------
    // Incorrect path.
    SimTK_TEST_MUST_THROW(top.getComponent<A>("oops/a2"));
    SimTK_TEST_MUST_THROW(b1->getComponent("/nonexistent"));
    // Wrong type.
    SimTK_TEST_MUST_THROW(top.getComponent<B>("a1/a2"));
    // Going too high up.
    SimTK_TEST_MUST_THROW(top.getComponent<A>(".."));
    SimTK_TEST_MUST_THROW(top.getComponent<A>("../"));
    SimTK_TEST_MUST_THROW(top.getComponent<A>("../.."));
    SimTK_TEST_MUST_THROW(top.getComponent<A>("../../"));
    SimTK_TEST_MUST_THROW(a1->getComponent<A>("../../"));
    SimTK_TEST_MUST_THROW(b2->getComponent<A>("../../../"));

    // Repeated name bug.
    // ------------------
    // There used to be a bug where calling traversePathToComponent({"tx/tx"})
    // would return the component at "tx" rather than the one at "tx/tx".
    A* atx = new A("tx");
    top.addComponent(atx);
    B* btx = new B("tx");
    atx->addComponent(btx);
    SimTK_TEST(&top.getComponent<Component>("tx/tx") == btx);
}

TEST_CASE("Component Interface Component::getStateVariableValue")
{
    TheWorld top;
    top.setName("top");
    Sub* a = new Sub();
    a->setName("a");
    Sub* b = new Sub();
    b->setName("b");

    top.add(a);
    a->addComponent(b);

    MultibodySystem system;
    top.buildUpSystem(system);
    State s = system.realizeTopology();

    SimTK_TEST(s.getNY() == 3);
    s.updY()[0] = 10; // "top/internalSub/subState"
    s.updY()[1] = 20; // "top/a/subState"
    s.updY()[2] = 30; // "top/a/b/subState"

    SimTK_TEST(top.getStateVariableValue(s, "internalSub/subState") == 10);
    SimTK_TEST(top.getStateVariableValue(s, "a/subState") == 20);
    SimTK_TEST(top.getStateVariableValue(s, "a/b/subState") == 30);
    SimTK_TEST(a->getStateVariableValue(s, "subState") == 20);
    SimTK_TEST(a->getStateVariableValue(s, "b/subState") == 30);
    SimTK_TEST(b->getStateVariableValue(s, "subState") == 30);
    SimTK_TEST(b->getStateVariableValue(s, "../subState") == 20);
    SimTK_TEST(b->getStateVariableValue(s, "../../internalSub/subState") == 10);

    SimTK_TEST_MUST_THROW_EXC(
            top.getStateVariableValue(s, "typo/b/subState"),
            OpenSim::Exception);
}

TEST_CASE("Component Interface getStateVariableValue with Component Path")
{
    using CP = ComponentPath;

    TheWorld top;
    top.setName("top");
    Sub* a = new Sub();
    a->setName("a");
    Sub* b = new Sub();
    b->setName("b");

    top.add(a);
    a->addComponent(b);

    MultibodySystem system;
    top.buildUpSystem(system);
    State s = system.realizeTopology();

    SimTK_TEST(s.getNY() == 3);
    s.updY()[0] = 10; // "top/internalSub/subState"
    s.updY()[1] = 20; // "top/a/subState"
    s.updY()[2] = 30; // "top/a/b/subState"

    SimTK_TEST(top.getStateVariableValue(s, CP{"internalSub/subState"}) == 10);
    SimTK_TEST(top.getStateVariableValue(s, CP{"a/subState"}) == 20);
    SimTK_TEST(top.getStateVariableValue(s, CP{"a/b/subState"}) == 30);
    SimTK_TEST(a->getStateVariableValue(s, CP{"subState"}) == 20);
    SimTK_TEST(a->getStateVariableValue(s, CP{"b/subState"}) == 30);
    SimTK_TEST(b->getStateVariableValue(s, CP{"subState"}) == 30);
    SimTK_TEST(b->getStateVariableValue(s, CP{"../subState"}) == 20);
    SimTK_TEST(b->getStateVariableValue(s, CP{"../../internalSub/subState"}) == 10);

    SimTK_TEST_MUST_THROW_EXC(
            top.getStateVariableValue(s, CP{"typo/b/subState"}),
            OpenSim::Exception);
}


TEST_CASE("Component Interface Component::resolveVariableNameAndOwner")
{
    TheWorld top;
    top.setName("top");
    Sub* a = new Sub();
    a->setName("a");
    Sub* b = new Sub();
    b->setName("b");

    top.add(a);
    a->addComponent(b);

    MultibodySystem system;
    top.buildUpSystem(system);
    State s = system.realizeTopology();

    const Component& internSub = top.getComponent("/internalSub");
    const Component* owner;

    SECTION("State Variables") {
        // Get the paths of all state variables under "top"
        const Array<std::string>& svPaths = top.getStateVariableNames();
        REQUIRE(svPaths.size() == 3);
        REQUIRE(svPaths[0] == "/internalSub/subState");
        REQUIRE(svPaths[1] == "/a/subState");
        REQUIRE(svPaths[2] == "/a/b/subState");

        // Verify correct execution
        std::string svNameCorrect("subState");
        std::string svName;

        // caller is top
        owner = top.resolveVariableNameAndOwner(
            ComponentPath("/internalSub/subState"), svName);
        CHECK(svName == svNameCorrect);
        CHECK(owner == &internSub);
        owner = top.resolveVariableNameAndOwner(
            ComponentPath("/a/subState"), svName);
        CHECK(svName == svNameCorrect);
        CHECK(owner == a);
        owner = top.resolveVariableNameAndOwner(
            ComponentPath("/a/b/subState"), svName);
        CHECK(svName == svNameCorrect);
        CHECK(owner == b);
        // caller is a
        owner = a->resolveVariableNameAndOwner(
            ComponentPath("/internalSub/subState"), svName);
        CHECK(svName == svNameCorrect);
        CHECK(owner == &internSub);
        owner = a->resolveVariableNameAndOwner(
            ComponentPath("/a/subState"), svName);
        CHECK(svName == svNameCorrect);
        CHECK(owner == a);
        owner = a->resolveVariableNameAndOwner(
            ComponentPath("/a/b/subState"), svName);
        CHECK(svName == svNameCorrect);
        CHECK(owner == b);
        // caller is b
        owner = b->resolveVariableNameAndOwner(
            ComponentPath("/internalSub/subState"), svName);
        CHECK(svName == svNameCorrect);
        CHECK(owner == &internSub);
        owner = b->resolveVariableNameAndOwner(
            ComponentPath("/a/subState"), svName);
        CHECK(svName == svNameCorrect);
        CHECK(owner == a);
        owner = b->resolveVariableNameAndOwner(
            ComponentPath("/a/b/subState"), svName);
        CHECK(svName == svNameCorrect);
        CHECK(owner == b);
        // ----- With relative paths, the caller matters.
        // down from a
        owner = a->resolveVariableNameAndOwner(
            ComponentPath("subState"), svName);
        CHECK(svName == svNameCorrect);
        CHECK(owner == a);
        owner = a->resolveVariableNameAndOwner(
            ComponentPath("b/subState"), svName);
        CHECK(svName == svNameCorrect);
        CHECK(owner == b);
        // down from b
        owner = b->resolveVariableNameAndOwner(
            ComponentPath("subState"), svName);
        CHECK(svName == svNameCorrect);
        CHECK(owner == b);
        // up from b
        owner = b->resolveVariableNameAndOwner(
            ComponentPath("../subState"), svName);
        CHECK(svName == svNameCorrect);
        CHECK(owner == a);
        owner = b->resolveVariableNameAndOwner(
            ComponentPath("../../internalSub/subState"), svName);
        CHECK(svName == svNameCorrect);
        CHECK(owner == &internSub);
    }

    SECTION("Discrete Variables") {
        // Get the paths of all discrete variables under "top"
        const Array<std::string>& dvPaths = top.getDiscreteVariableNames();
        REQUIRE(dvPaths.size() == 3);
        REQUIRE(dvPaths[0] == "/internalSub/dvX");
        REQUIRE(dvPaths[1] == "/a/dvX");
        REQUIRE(dvPaths[2] == "/a/b/dvX");

        // Check a VariableNotFound exception
        CHECK_THROWS_AS(top.setModelingOption(s, "/a/should_not_be_found", 0),
            VariableNotFound);

        // Verify correct execution
        std::string dvNameCorrect("dvX");
        std::string dvName;
        // ----- With an absolute path, the caller doesn't matter.
        // caller is top
        owner = top.resolveVariableNameAndOwner(
            ComponentPath("/internalSub/dvX"), dvName);
        CHECK(dvName == dvNameCorrect);
        CHECK(owner == &internSub);
        owner = top.resolveVariableNameAndOwner(ComponentPath("/a/dvX"),
            dvName);
        CHECK(dvName == dvNameCorrect);
        CHECK(owner == a);
        owner = top.resolveVariableNameAndOwner(ComponentPath("/a/b/dvX"),
            dvName);
        CHECK(dvName == dvNameCorrect);
        CHECK(owner == b);
        // caller is a
        owner = a->resolveVariableNameAndOwner(
            ComponentPath("/internalSub/dvX"), dvName);
        CHECK(dvName == dvNameCorrect);
        CHECK(owner == &internSub);
        owner = a->resolveVariableNameAndOwner(ComponentPath("/a/dvX"),
            dvName);
        CHECK(dvName == dvNameCorrect);
        CHECK(owner == a);
        owner = a->resolveVariableNameAndOwner(ComponentPath("/a/b/dvX"),
            dvName);
        CHECK(dvName == dvNameCorrect);
        CHECK(owner == b);
        // caller is b
        owner = b->resolveVariableNameAndOwner(
            ComponentPath("/internalSub/dvX"), dvName);
        CHECK(dvName == dvNameCorrect);
        CHECK(owner == &internSub);
        owner = b->resolveVariableNameAndOwner(ComponentPath("/a/dvX"),
            dvName);
        CHECK(dvName == dvNameCorrect);
        CHECK(owner == a);
        owner = b->resolveVariableNameAndOwner(ComponentPath("/a/b/dvX"),
            dvName);
        CHECK(dvName == dvNameCorrect);
        CHECK(owner == b);
        // ----- With relative paths, the caller matters.
        // down from a
        owner = a->resolveVariableNameAndOwner(ComponentPath("dvX"),
            dvName);
        CHECK(dvName == dvNameCorrect);
        CHECK(owner == a);
        owner = a->resolveVariableNameAndOwner(ComponentPath("b/dvX"),
            dvName);
        CHECK(dvName == dvNameCorrect);
        CHECK(owner == b);
        // down from b
        owner = b->resolveVariableNameAndOwner(ComponentPath("dvX"),
            dvName);
        CHECK(dvName == dvNameCorrect);
        CHECK(owner == b);
        // up from b
        owner = b->resolveVariableNameAndOwner(ComponentPath("../dvX"),
            dvName);
        CHECK(dvName == dvNameCorrect);
        CHECK(owner == a);
        owner = b->resolveVariableNameAndOwner(
            ComponentPath("../../internalSub/dvX"), dvName);
        CHECK(dvName == dvNameCorrect);
        CHECK(owner == &internSub);
    }

    SECTION("Exceptions are thrown appropriately") {
        std::string dvName;
        CHECK_THROWS_AS(
            owner = top.resolveVariableNameAndOwner(
                ComponentPath(""), dvName),
                EmptyComponentPath);
        CHECK_THROWS_AS(
            owner = top.resolveVariableNameAndOwner(
                ComponentPath("/typoSub/dvX"), dvName),
                VariableOwnerNotFoundOnSpecifiedPath);
        owner = a->resolveVariableNameAndOwner(
            ComponentPath("discVarTypo"), dvName);
        CHECK_THROWS_AS(
            owner->setDiscreteVariableValue(s, dvName, 3.1415),
            VariableNotFound);
        double value;
        CHECK_THROWS_AS(
            value = owner->getDiscreteVariableValue(s, dvName),
            VariableNotFound);
    }
}

TEST_CASE("Component Interface Modeling Options")
{
    TheWorld top;
    top.setName("top");
    Sub* a = new Sub();
    a->setName("a");
    Sub* b = new Sub();
    b->setName("b");

    top.add(a);
    a->addComponent(b);

    MultibodySystem system;
    top.buildUpSystem(system);
    State s = system.realizeTopology();

    // Get the paths of all discrete variables under "top"
    const Array<std::string>& moPaths = top.getModelingOptionNames();
    SECTION("Paths are correct") {
        REQUIRE(moPaths.size() == 3);
        REQUIRE(moPaths[0] == "/internalSub/moX");
        REQUIRE(moPaths[1] == "/a/moX");
        REQUIRE(moPaths[2] == "/a/b/moX");
    }

    // Check a VariableNotFound exceptions
    CHECK_THROWS_AS(top.setModelingOption(s, "/a/should_not_be_found", 0),
        VariableNotFound);
    CHECK_THROWS_AS(top.getModelingOption(s, "/a/should_not_be_found"),
        VariableNotFound);

    // Set different values for moX at the different paths
    top.setModelingOption(s, moPaths[0], 0);
    top.setModelingOption(s, moPaths[1], 1);
    top.setModelingOption(s, moPaths[2], 2);
    CHECK_THROWS_AS(top.setModelingOption(s, moPaths[2], 3),
        ModelingOptionMaxExceeded);

    SECTION("Getters") {
        // ----- With an absolute path, the caller doesn't matter.
        // caller is top
        CHECK(top.getModelingOption(s,"/internalSub/moX") == 0);
        CHECK(top.getModelingOption(s,"/a/moX") == 1);
        CHECK(top.getModelingOption(s,"/a/b/moX") == 2);
        // caller is a
        CHECK(a->getModelingOption(s,"/internalSub/moX") == 0);
        CHECK(a->getModelingOption(s,"/a/moX") == 1);
        CHECK(a->getModelingOption(s,"/a/b/moX") == 2);
        // caller is b
        CHECK(b->getModelingOption(s,"/internalSub/moX") == 0);
        CHECK(b->getModelingOption(s,"/a/moX") == 1);
        CHECK(b->getModelingOption(s,"/a/b/moX") == 2);
        // ----- With relative paths, the caller matters.
        // down from a
        CHECK(a->getModelingOption(s,"moX") == 1);
        CHECK(a->getModelingOption(s,"b/moX") == 2);
        // down from b
        CHECK(b->getModelingOption(s,"/moX") == 2);
        // up from b
        CHECK(b->getModelingOption(s,"../moX") == 1);
        CHECK(b->getModelingOption(s,"../../internalSub/moX") == 0);
    }
}

TEST_CASE("Component Interface Discrete Variables")
{
    TheWorld top;
    top.setName("top");
    Sub* a = new Sub();
    a->setName("a");
    Sub* b = new Sub();
    b->setName("b");

    top.add(a);
    a->addComponent(b);

    MultibodySystem system;
    top.buildUpSystem(system);
    State s = system.realizeTopology();

    // Get the paths of all discrete variables under "top"
    const Array<std::string>& dvPaths = top.getDiscreteVariableNames();
    SECTION("Paths are correct") {
        REQUIRE(dvPaths.size() == 3);
        REQUIRE(dvPaths[0] == "/internalSub/dvX");
        REQUIRE(dvPaths[1] == "/a/dvX");
        REQUIRE(dvPaths[2] == "/a/b/dvX");
    }

    // Set different values for dvX at the different paths
    top.setDiscreteVariableValue(s, dvPaths[0], 0.0);
    top.setDiscreteVariableValue(s, dvPaths[1], 10.0);
    top.setDiscreteVariableValue(s, dvPaths[2], 20.0);

    SECTION("Getters") {
        // ----- With an absolute path, the caller doesn't matter.
        // caller is top
        CHECK(top.getDiscreteVariableValue(s,"/internalSub/dvX") == 0.0);
        CHECK(top.getDiscreteVariableValue(s,"/a/dvX") == 10.0);
        CHECK(top.getDiscreteVariableValue(s,"/a/b/dvX") == 20.0);
        // caller is a
        CHECK(a->getDiscreteVariableValue(s,"/internalSub/dvX") == 0.0);
        CHECK(a->getDiscreteVariableValue(s,"/a/dvX") == 10.0);
        CHECK(a->getDiscreteVariableValue(s,"/a/b/dvX") == 20.0);
        // caller is b
        CHECK(b->getDiscreteVariableValue(s,"/internalSub/dvX") == 0.0);
        CHECK(b->getDiscreteVariableValue(s,"/a/dvX") == 10.0);
        CHECK(b->getDiscreteVariableValue(s,"/a/b/dvX") == 20.0);
        // ----- With relative paths, the caller matters.
        // down from a
        CHECK(a->getDiscreteVariableValue(s,"dvX") == 10.0);
        CHECK(a->getDiscreteVariableValue(s,"b/dvX") == 20.0);
        // down from b
        CHECK(b->getDiscreteVariableValue(s,"/dvX") == 20.0);
        // up from b
        CHECK(b->getDiscreteVariableValue(s,"../dvX") == 10.0);
        CHECK(b->getDiscreteVariableValue(s,"../../internalSub/dvX") == 0.0);
    }
}

TEST_CASE("Component Interface Discrete Variables Vec3")
{
    // ------------------------------------------------------------------------
    // Component Bar possesses a discrete variable called "point", which is
    // a Vec3. Additionally, point was allocated as though it were a member
    // of a pre-existing Simbody object (e.g., it was allocated from the
    // GeneralForceSubsystem and not from the DefaultSybsystem like most, if
    // not all, discrete variables allocated in OpenSim).
    // This test case
    // 1) checks the interface for handling discrete variables that are not
    //    type double, and
    // 2) verifies that OpenSim can properly wrap discrete state variables
    //    when those variables are allocated externally (i.e., are not
    //    allocated by calling Component::addDiscreteVariable()).
    // ------------------------------------------------------------------------

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

    bar.connectSocket_parentFoo(foo);
    bar.connectSocket_childFoo(foo2);

    theWorld.connect();
    theWorld.buildUpSystem(system);

    State s = system.realizeTopology();

    // Get the paths of all discrete variables under "theWorld"
    const Array<std::string>& dvPaths = theWorld.getDiscreteVariableNames();
    SECTION("Paths are correct") {
        REQUIRE(dvPaths.size() == 2);
        REQUIRE(dvPaths[0] == "/internalSub/dvX");
        REQUIRE(dvPaths[1] == "/Bar/point");
    }

    // Get the starting value of point
    // The starting value should be (0.0, 0.1, 0.2).
    // See Bar::extendRealizeTopology().
    Vec3 pointStart(0.0, 0.1, 0.2);
    Vec3 point = theWorld.getDiscreteVariableValue<Vec3>(s, "/Bar/point");
    REQUIRE(point == pointStart);

    // Verify that changes to the local variable point do not change
    // the value of the discrete variable. That is, verify that the local
    // variable point holds a copy of the data.
    point *= 10.0;
    Vec3 point2 = theWorld.getDiscreteVariableValue<Vec3>(s, "/Bar/point");
    REQUIRE(point2 != point);
    REQUIRE(point2 == pointStart);

    // Set a new value and check it.
    theWorld.setDiscreteVariableValue<Vec3>(s, "/Bar/point", point);
    Vec3 point3 = theWorld.getDiscreteVariableValue<Vec3>(s, "/Bar/point");
    REQUIRE(point3 == point);
}

TEST_CASE("Component Interface State Trajectories")
{
    MultibodySystem system;
    TheWorld wrld;
    wrld.setName("World");

    Foo& foo = *new Foo();
    foo.setName("Foo");
    wrld.add(&foo);
    foo.set_mass(2.0);

    Foo& foo2 = *new Foo();
    foo2.setName("Foo2");
    foo2.set_mass(3.0);
    wrld.add(&foo2);

    Bar& bar = *new Bar();
    bar.setName("Bar");
    wrld.add(&bar);

    bar.connectSocket_parentFoo(foo);
    bar.connectSocket_childFoo(foo2);

    wrld.connect();
    wrld.buildUpSystem(system);

    State s = system.realizeTopology();

    // Form the q and u vectors
    const Array<std::string>& svPaths = wrld.getStateVariableNames();
    REQUIRE(svPaths.size() == 4);
    const Vector y = Vector(svPaths.size(), 0.1);

    // Get the paths of all discrete variables under "wrld"
    const OpenSim::Array<std::string>& dvPaths =
        wrld.getDiscreteVariableNames();
    REQUIRE(dvPaths.size() == 2);
    REQUIRE(dvPaths[0] == "/internalSub/dvX");
    REQUIRE(dvPaths[1] == "/Bar/point");

    // Get the paths of all modeling moptions under "wrld"
    const OpenSim::Array<std::string>& moPaths =
        wrld.getModelingOptionNames();
    REQUIRE(moPaths.size() == 2);
    REQUIRE(moPaths[0] == "/internalSub/moX");
    REQUIRE(moPaths[1] == "/Bar/moY");

    // Get the starting value of point
    // The starting value should be (0.0, 0.1, 1.0).
    // See Bar::extendRealizeTopology().
    Vec3 pointStart = wrld.getDiscreteVariableValue<Vec3>(s, "/Bar/point");

    // Run an artificial simulation and record the state trajectory
    SimTK::Array_<SimTK::State> simTraj;
    int nsteps{11};
    simTraj.reserve(nsteps);
    int moX{0}, moY{0};
    double dvX{0.0};
    Vec3 point(0.0);
    for (int i = 0; i < nsteps; ++i){
        // Time
        s.updTime() = i*0.01;
        // State Variables
        wrld.setStateVariableValue(s, svPaths[0], i*y[0] + 0);
        wrld.setStateVariableValue(s, svPaths[1], i*y[1] + 1);
        wrld.setStateVariableValue(s, svPaths[2], i*y[2] + 2);
        wrld.setStateVariableValue(s, svPaths[3], i*y[3] + 3);
        // Discrete Variables
        dvX = i*0.5 ;
        wrld.setDiscreteVariableValue(s, dvPaths[0], dvX);
        point = i*pointStart;
        wrld.setDiscreteVariableValue<Vec3>(s, dvPaths[1], point);
        // Modeling Options
        moX = i % 2;
        wrld.setModelingOption(s, moPaths[0], moX);
        moY = (i+1) % 2;
        wrld.setModelingOption(s, moPaths[1], moY);
        // Accumulate the simulated state trajectory
        system.realize(s, Stage::Report);
        simTraj.emplace_back(s);
    }

    // Extract individual variable trajectories (as though serializing)
    // state variables
    SimTK::Array_<double> y0Traj, y1Traj, y2Traj, y3Traj;
    wrld.getStateVariableTrajectory<double>(svPaths[0], simTraj, y0Traj);
    wrld.getStateVariableTrajectory<double>(svPaths[1], simTraj, y1Traj);
    wrld.getStateVariableTrajectory<double>(svPaths[2], simTraj, y2Traj);
    wrld.getStateVariableTrajectory<double>(svPaths[3], simTraj, y3Traj);
    // discrete variables
    SimTK::Array_<double> dv0Traj;
    SimTK::Array_<Vec3> dv1Traj;
    wrld.getDiscreteVariableTrajectory<double>(dvPaths[0], simTraj, dv0Traj);
    wrld.getDiscreteVariableTrajectory<Vec3>(dvPaths[1], simTraj, dv1Traj);
    // modeling options
    SimTK::Array_<int> mo0Traj;
    wrld.getModelingOptionTrajectory<int>(moPaths[0], simTraj, mo0Traj);
    SimTK::Array_<int> mo1Traj;
    wrld.getModelingOptionTrajectory<int>(moPaths[1], simTraj, mo1Traj);

    // Check the individual variable trajectories
    for (int i = 0; i < nsteps; ++i){
        // state variables
        CHECK(y0Traj[i] == i*y[0] + 0);
        CHECK(y1Traj[i] == i*y[1] + 1);
        CHECK(y2Traj[i] == i*y[2] + 2);
        CHECK(y3Traj[i] == i*y[3] + 3);
        // discrete variables
        CHECK(dv0Traj[i] == i*0.5);
        CHECK(dv1Traj[i] == i*pointStart);
        // modeling options
        CHECK(mo0Traj[i] == (i%2));
        CHECK(mo1Traj[i] == ((i+1)%2));
    }

    // Create a new state trajectory (as though deserializing)
    // newTraj must be the expected size before any set calls.
    SimTK::Array_<SimTK::State> newTraj;
    for (int i = 0; i < nsteps; ++i) newTraj.emplace_back(s);
    // state variables
    wrld.setStateVariableTrajectory<double>(svPaths[0], y0Traj, newTraj);
    wrld.setStateVariableTrajectory<double>(svPaths[1], y1Traj, newTraj);
    wrld.setStateVariableTrajectory<double>(svPaths[2], y2Traj, newTraj);
    wrld.setStateVariableTrajectory<double>(svPaths[3], y3Traj, newTraj);
    // discrete variables
    wrld.setDiscreteVariableTrajectory<double>(dvPaths[0], dv0Traj, newTraj);
    wrld.setDiscreteVariableTrajectory<Vec3>(dvPaths[1], dv1Traj, newTraj);
    // modeling option
    wrld.setModelingOptionTrajectory<int>(moPaths[0], mo0Traj, newTraj);
    wrld.setModelingOptionTrajectory<int>(moPaths[1], mo1Traj, newTraj);

    // Check the new state trajectory
    SimTK::Array_<double> nq0Traj, nq1Traj, nq2Traj, nq3Traj;
    SimTK::Array_<double> ndv0Traj;
    SimTK::Array_<Vec3> ndv1Traj;
    SimTK::Array_<int> nmo0Traj;
    SimTK::Array_<int> nmo1Traj;
    wrld.getStateVariableTrajectory<double>(svPaths[0], newTraj, nq0Traj);
    wrld.getStateVariableTrajectory<double>(svPaths[1], newTraj, nq1Traj);
    wrld.getStateVariableTrajectory<double>(svPaths[2], newTraj, nq2Traj);
    wrld.getStateVariableTrajectory<double>(svPaths[3], newTraj, nq3Traj);
    wrld.getDiscreteVariableTrajectory<double>(dvPaths[0], newTraj, ndv0Traj);
    wrld.getDiscreteVariableTrajectory<Vec3>(dvPaths[1], newTraj, ndv1Traj);
    wrld.getModelingOptionTrajectory<int>(moPaths[0], newTraj, nmo0Traj);
    wrld.getModelingOptionTrajectory<int>(moPaths[1], newTraj, nmo1Traj);
    for (int i = 0; i < nsteps; ++i){
        CHECK(nq0Traj[i] == i*y[0] + 0);
        CHECK(nq1Traj[i] == i*y[1] + 1);
        CHECK(nq2Traj[i] == i*y[2] + 2);
        CHECK(nq3Traj[i] == i*y[3] + 3);
        CHECK(ndv0Traj[i] == i*0.5);
        CHECK(ndv1Traj[i] == i*pointStart);
        CHECK(nmo0Traj[i] == (i%2));
        CHECK(nmo1Traj[i] == ((i+1)%2));
    }
}

TEST_CASE("Component Interface Input/Output Connections")
{
    {
        TheWorld world;
        Foo* foo1 = new Foo();
        Foo* foo2 = new Foo();
        Bar* bar = new Bar();

        foo1->setName("foo1");
        foo2->setName("foo2");
        bar->setName("bar");
        bar->connectSocket_parentFoo(*foo1);
        bar->connectSocket_childFoo(*foo2);

        world.add(foo1);
        world.add(foo2);
        world.add(bar);

        MultibodySystem mbs;

        world.connect();

        // do any other input/output connections
        foo1->connectInput_input1(bar->getOutput("PotentialEnergy"));

        // Test various exceptions for inputs, outputs, sockets
        ASSERT_THROW(InputNotFound, foo1->getInput("input0"));
        ASSERT_THROW(SocketNotFound, bar->updSocket<Foo>("parentFoo0"));
        ASSERT_THROW(OutputNotFound,
            world.getComponent("./internalSub").getOutput("subState0"));
        // Ensure that getOutput does not perform a "find"
        ASSERT_THROW(OutputNotFound,
            world.getOutput("./internalSub/subState"));

        foo2->connectInput_input1(world.getComponent("./internalSub").getOutput("subState"));

        foo1->connectInput_AnglesIn(foo2->getOutput("Qs"));
        foo2->connectInput_AnglesIn(foo1->getOutput("Qs"));

        foo1->connectInput_activation(bar->getOutput("activation"));
        foo1->connectInput_fiberLength(bar->getOutput("fiberLength"));

        foo2->connectInput_activation(bar->getOutput("activation"));
        foo2->connectInput_fiberLength(bar->getOutput("fiberLength"));

        world.connect();
        world.buildUpSystem(mbs);
    }
    // Test exception message when asking for the value of an input that is
    // not wired up.
    class A : public Component { // Test single-value input.
        OpenSim_DECLARE_CONCRETE_OBJECT(A, Component);
    public:
        OpenSim_DECLARE_INPUT(in1, double, SimTK::Stage::Model, "");
    };
    class B : public Component { // Test list inputs.
        OpenSim_DECLARE_CONCRETE_OBJECT(B, Component);
    public:
        OpenSim_DECLARE_LIST_INPUT(in1, double, SimTK::Stage::Model, "");
    };
    class C : public Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(C, Component);
    public:
        OpenSim_DECLARE_OUTPUT(out1, double, calcOut1, SimTK::Stage::Time);
        double calcOut1(const SimTK::State& state) const { return 0; }
    };
    {
        // Single-value input.
        TheWorld world;
        A* a = new A(); a->setName("a");
        world.add(a);
        MultibodySystem system;
        world.connect();
        world.buildUpSystem(system);
        State s = system.realizeTopology();
        system.realize(s, Stage::Model);
        SimTK_TEST_MUST_THROW_EXC(a->getInput<double>("in1").getValue(s),
                InputNotConnected);
    }
    {
        // List input.
        // We must wire up an output to the input, as a list input with no
        // connectees is always "connected."
        TheWorld world;
        B* b = new B(); b->setName("b");
        C* c = new C(); c->setName("c");
        world.add(b);
        world.add(c);
        b->updInput("in1").connect(c->getOutput("out1"));
        MultibodySystem system;
        world.connect();
        world.buildUpSystem(system);
        State s = system.realizeTopology();
        system.realize(s, Stage::Model);
        // The following will work, now that the connection is satisfied.
        b->getInput<double>("in1").getValue(s, 0);
        // Disconnect to get the "not connected"exception.
        b->clearConnections();
        SimTK_TEST_MUST_THROW_EXC(b->getInput<double>("in1").getValue(s, 0),
                InputNotConnected);
    }
}

TEST_CASE("Component Interface Input Connectee Paths")
{
    {
        std::string componentPath, outputName, channelName, alias;
        AbstractInput::parseConnecteePath("/foo/bar|output",
                                          componentPath, outputName,
                                          channelName, alias);
        SimTK_TEST(componentPath == "/foo/bar");
        SimTK_TEST(outputName == "output");
        SimTK_TEST(channelName == "");
        SimTK_TEST(alias == "");
    }
    {
        std::string componentPath, outputName, channelName, alias;
        AbstractInput::parseConnecteePath("/foo/bar|output:channel",
                                          componentPath, outputName,
                                          channelName, alias);
        SimTK_TEST(componentPath == "/foo/bar");
        SimTK_TEST(outputName == "output");
        SimTK_TEST(channelName == "channel");
        SimTK_TEST(alias == "");
    }
    {
        std::string componentPath, outputName, channelName, alias;
        AbstractInput::parseConnecteePath("/foo/bar|output(baz)",
                                          componentPath, outputName,
                                          channelName, alias);
        SimTK_TEST(componentPath == "/foo/bar");
        SimTK_TEST(outputName == "output");
        SimTK_TEST(channelName == "");
        SimTK_TEST(alias == "baz");
    }
    {
        std::string componentPath, outputName, channelName, alias;
        AbstractInput::parseConnecteePath("/foo/bar|output:channel(baz)",
                                          componentPath, outputName,
                                          channelName, alias);
        SimTK_TEST(componentPath == "/foo/bar");
        SimTK_TEST(outputName == "output");
        SimTK_TEST(channelName == "channel");
        SimTK_TEST(alias == "baz");
    }

    // TODO test invalid names as well.
}

TEST_CASE("Component Interface Exceptions when Connectee Type Mismatches")
{
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
    // This class has a socket.
    class C : public Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(C, Component);
    public:
        OpenSim_DECLARE_SOCKET(socket1, A, "");
    };

    // Test various type mismatches.
    // -----------------------------
    // First, check for exceptions when directly connecting inputs to outputs
    // (or sockets to connectees).
    { // Socket.
        TheWorld model;
        B* b = new B(); b->setName("b");
        C* c = new C(); c->setName("c");
        model.add(b); model.add(c);
        SimTK_TEST_MUST_THROW_EXC(c->updSocket("socket1").connect(*b),
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
    { // Socket.
        TheWorld model;
        B* b = new B(); b->setName("b");
        C* c = new C(); c->setName("c");
        model.add(b); model.add(c);
        c->updSocket("socket1").setConnecteePath("../b");
        SimTK_TEST_MUST_THROW_EXC(model.connect(), OpenSim::Exception);
    }
    { // single-value output -> single-value input.
        TheWorld model;
        A* a = new A(); a->setName("a");
        B* b = new B(); b->setName("b");
        model.add(a); model.add(b);
        b->updInput("in1").setConnecteePath("../a/out1");
        SimTK_TEST_MUST_THROW_EXC(model.connect(), OpenSim::Exception);
    }
    { // single-value output -> list input.
        TheWorld model;
        A* a = new A(); a->setName("a");
        B* b = new B(); b->setName("b");
        model.add(a); model.add(b);
        b->updInput("inL").appendConnecteePath("../a/out1");
        SimTK_TEST_MUST_THROW_EXC(model.connect(), OpenSim::Exception);
    }
    { // list output -> single-value input.
        TheWorld model;
        A* a = new A(); a->setName("a");
        B* b = new B(); b->setName("b");
        model.add(a); model.add(b);
        b->updInput("in1").setConnecteePath("../a/outL:2");
        SimTK_TEST_MUST_THROW_EXC(model.connect(), OpenSim::Exception);
    }
    { // list output -> list input.
        TheWorld model;
        A* a = new A(); a->setName("a");
        B* b = new B(); b->setName("b");
        model.add(a); model.add(b);
        b->updInput("inL").appendConnecteePath("../a/outL:0");
        SimTK_TEST_MUST_THROW_EXC(model.connect(), OpenSim::Exception);
    }
}

TEST_CASE("Component Interface Throws Exceptions if Socket Name Exists Already")
{
    // Make sure that it is not possible for a class to have more than one
    // socket with a given name, even if the connectee types are different.

    // We will use Z and Y as the connectee types.
    class Z : public Component
    {   OpenSim_DECLARE_CONCRETE_OBJECT(Z, Component); };
    class Y : public Component
    {   OpenSim_DECLARE_CONCRETE_OBJECT(Y, Component); };

    // A is the base class that has a socket named 'socket1', of type Z.
    class A : public Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(A, Component);
    public:
        OpenSim_DECLARE_SOCKET(socket1, Z, "");
    };

    // BSame tries to reuse the name 'socket1', and also connect to type Z.
    class BSame : public A {
        OpenSim_DECLARE_CONCRETE_OBJECT(BSame, A);
    public:
        OpenSim_DECLARE_SOCKET(socket1, Z, "");
    };

    // BDifferent uses the same name 'socket1' but connects to a different type.
    class BDifferent : public A {
        OpenSim_DECLARE_CONCRETE_OBJECT(BDifferent, A);
    public:
        OpenSim_DECLARE_SOCKET(socket1, Y, "");
    };

    ASSERT_THROW_MSG(OpenSim::Exception,
            "BSame already has a socket named 'socket1'",
            BSame b;);
    ASSERT_THROW_MSG(OpenSim::Exception,
            "BDifferent already has a socket named 'socket1'",
            BDifferent b;);

    // The API user may try to create two sockets with the
    // same name in the same exact class (that is, not separated across the
    // inheritance hierarchy). We do not need to test this case, because it
    // leads to a compiling error (duplicate member variable).
}

TEST_CASE("Component Interface Throws Exception if Input Name Exists Already")
{
    // Make sure that it is not possible for a class to have more than one
    // input with a given name, even if the connectee types are different.

    { // Single-value input.
        class A : public Component {
            OpenSim_DECLARE_CONCRETE_OBJECT(A, Component);
        public:
            OpenSim_DECLARE_INPUT(in1, Vec3, SimTK::Stage::Model, "");
        };

        // BSame tries to reuse the name 'in1', and also connect to Vec3.
        class BSame : public A {
            OpenSim_DECLARE_CONCRETE_OBJECT(BSame, A);
        public:
            OpenSim_DECLARE_INPUT(in1, Vec3, SimTK::Stage::Model, "");
        };

        // BDifferent uses the same name 'in1' but connects to a different type.
        class BDifferent : public A {
            OpenSim_DECLARE_CONCRETE_OBJECT(BDifferent, A);
        public:
            OpenSim_DECLARE_INPUT(in1, double, SimTK::Stage::Model, "");
        };

        ASSERT_THROW_MSG(OpenSim::Exception,
                "BSame already has an input named 'in1'",
                BSame b;);
        ASSERT_THROW_MSG(OpenSim::Exception,
                "BDifferent already has an input named 'in1'",
                BDifferent b;);
    }

    { // List input.
        class A : public Component {
            OpenSim_DECLARE_CONCRETE_OBJECT(A, Component);
        public:
            OpenSim_DECLARE_LIST_INPUT(in1, Vec3, SimTK::Stage::Model, "");
        };

        // BSame tries to reuse the name 'in1', and also connect to Vec3.
        class BSame : public A {
            OpenSim_DECLARE_CONCRETE_OBJECT(BSame, A);
        public:
            OpenSim_DECLARE_LIST_INPUT(in1, Vec3, SimTK::Stage::Model, "");
        };

        // BDifferent uses the same name 'in1' but connects to a different type.
        class BDifferent : public A {
            OpenSim_DECLARE_CONCRETE_OBJECT(BDifferent, A);
        public:
            OpenSim_DECLARE_LIST_INPUT(in1, double, SimTK::Stage::Model, "");
        };

        ASSERT_THROW_MSG(OpenSim::Exception,
                "BSame already has an input named 'in1'",
                BSame b;);
        ASSERT_THROW_MSG(OpenSim::Exception,
                "BDifferent already has an input named 'in1'",
                BDifferent b;);
    }
}

TEST_CASE("Component Interface Throws Exceptions if Output Name Exists Already")
{
    // Make sure that it is not possible for a class to have more than one
    // output with a given name, even if the types are different.

    { // Single-value output.
        class A : public Component {
            OpenSim_DECLARE_CONCRETE_OBJECT(A, Component);
        public:
            OpenSim_DECLARE_OUTPUT(out1, double, calcOut1, SimTK::Stage::Time);
            double calcOut1(const SimTK::State& state) const { return 0; }
        };

        // BSame tries to reuse the name 'out1', and also uses type double.
        class BSame : public A {
            OpenSim_DECLARE_CONCRETE_OBJECT(BSame, A);
        public:
            OpenSim_DECLARE_OUTPUT(out1, double, calcOut1, SimTK::Stage::Time);
            double calcOut1(const SimTK::State& state) const { return 0; }
        };

        // BDifferent uses the same name 'out1' but uses a different type.
        class BDifferent : public A {
            OpenSim_DECLARE_CONCRETE_OBJECT(BDifferent, A);
        public:
            OpenSim_DECLARE_OUTPUT(out1, Vec3, calcOut1, SimTK::Stage::Time);
            Vec3 calcOut1(const SimTK::State& state) const { return Vec3(0); }
        };

        ASSERT_THROW_MSG(OpenSim::Exception,
                "BSame already has an output named 'out1'",
                BSame b;);
        ASSERT_THROW_MSG(OpenSim::Exception,
                "BDifferent already has an output named 'out1'",
                BDifferent b;);
    }

    { // List output.
        class A : public Component {
            OpenSim_DECLARE_CONCRETE_OBJECT(A, Component);
        public:
            OpenSim_DECLARE_LIST_OUTPUT(out1, double, calcOut1,
                                        SimTK::Stage::Time);
            double calcOut1(const SimTK::State& state,
                            const std::string&) const { return 0; }
        };

        // BSame tries to reuse the name 'out1', and also uses type double.
        class BSame : public A {
            OpenSim_DECLARE_CONCRETE_OBJECT(BSame, A);
        public:
            OpenSim_DECLARE_LIST_OUTPUT(out1, double, calcOut1,
                                        SimTK::Stage::Time);
            double calcOut1(const SimTK::State& state,
                            const std::string&) const { return 0; }
        };

        // BDifferent uses the same name 'out1' but uses a different type.
        class BDifferent : public A {
            OpenSim_DECLARE_CONCRETE_OBJECT(BDifferent, A);
        public:
            OpenSim_DECLARE_LIST_OUTPUT(out1, Vec3, calcOut1,
                                        SimTK::Stage::Time);
            Vec3 calcOut1(const SimTK::State& state,
                          const std::string&) const { return Vec3(0); }
        };

        ASSERT_THROW_MSG(OpenSim::Exception,
                "BSame already has an output named 'out1'",
                BSame b;);
        ASSERT_THROW_MSG(OpenSim::Exception,
                "BDifferent already has an output named 'out1'",
                BDifferent b;);
    }
}

template<typename RowVec>
void assertEqual(const RowVec& a, const RowVec& b) {
    ASSERT(a.nrow() == b.nrow());
    ASSERT(a.ncol() == b.ncol());
    for(int i = 0; i < a.ncol(); ++i)
        ASSERT_EQUAL(a[i], b[i], 1e-10);
}

TEST_CASE("Component Interface Table Source")
{
    using namespace OpenSim;
    using namespace SimTK;

    {
        TheWorld model{};
        auto tablesource = new TableSourceVec3{};
        tablesource->setName("tablesource");
        tablesource->set_filename("dataWithEformat.trc");
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

    tableReporter->addToReport(tableSource->getOutput("column"));

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

TEST_CASE("Component Interface TableReporter Usage")
{
    // TableReporter works fine even if its input has no connectees.
    {
        TheWorld model;
        auto* reporter = new TableReporter();
        reporter->set_report_time_interval(0.1);
        model.addComponent(reporter);

        MultibodySystem system;
        model.buildUpSystem(system);

        {
            SimTK::State s = system.realizeTopology();
            RungeKuttaFeldbergIntegrator integ(system);
            integ.setAccuracy(1.0e-3);
            TimeStepper ts(system, integ);
            ts.initialize(s);
            ts.stepTo(1.0);
            const auto& table = reporter->getTable();
            std::cout << "TableReporter table after simulating:\n"
                      << table.toString() << std::endl;
            SimTK_TEST_MUST_THROW_EXC(table.getDependentColumnAtIndex(0),
                                      EmptyTable);
        }

        // Ensure that clearing the table and performing a new simulation works
        // even if the reporter's input has no connectees.
        reporter->clearTable();
        std::cout << "TableReporter table after clearing:\n"
                  << reporter->getTable().toString() << std::endl;
        SimTK_TEST_MUST_THROW_EXC(
            reporter->getTable().getDependentColumnAtIndex(0),
            EmptyTable);

        {
            SimTK::State s = system.realizeTopology();
            RungeKuttaFeldbergIntegrator integ(system);
            integ.setAccuracy(1.0e-3);
            TimeStepper ts(system, integ);
            ts.initialize(s);
            ts.stepTo(1.0);
            const auto& table = reporter->getTable();
            std::cout << "TableReporter table after simulating again:\n"
                      << table.toString() << std::endl;
            SimTK_TEST_MUST_THROW_EXC(table.getDependentColumnAtIndex(0),
                                      EmptyTable);
        }
    }
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

TEST_CASE("Component Interface List Input Connectee Serialization Behavior")
{
    // We build a model, store the input connectee paths, then
    // recreate the same model from a serialization, and make sure the
    // connectee paths are the same.

    writeTimeSeriesTableForInputConnecteeSerialization();

    // Helper function.
    auto getConnecteePaths = [](const AbstractInput& in) {
        const auto numConnectees = in.getNumConnectees();
        std::vector<std::string> connecteePaths(numConnectees);
        for (unsigned ic = 0u; ic < numConnectees; ++ic) {
            connecteePaths[ic] = in.getConnecteePath(ic);
        }
        return connecteePaths;
    };

    // Build a model and serialize it.
    std::string modelFileName = "testComponentInterface_"
                                "testListInputConnecteeSerialization_world.xml";
    std::vector<std::string> expectedConnecteePaths{
            "/producer|column:a",
            "/producer|column:c",
            "/producer|column:b(berry)"};
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
        reporter->addToReport(output.getChannel("a"));
        reporter->addToReport(output.getChannel("c"));
        // We want to make sure aliases are preserved.
        reporter->addToReport(output.getChannel("b"), "berry");
        // Ensure that re-finalizing from properties does not cause Inputs
        // to hold onto stale references to the outputs' channels.
        world.finalizeFromProperties();
        world.connect();
        MultibodySystem system;
        world.buildUpSystem(system);

        // Grab the connectee paths.
        const auto& input = reporter->getInput("inputs");
        SimTK_TEST(getConnecteePaths(input) == expectedConnecteePaths);

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
        SimTK_TEST(input.isListSocket());
        // Check connectee paths before *and* after connecting, since
        // the connecting process edits the connectee_name property.
        SimTK_TEST(getConnecteePaths(input) == expectedConnecteePaths);
        world.connect();
        SimTK_TEST(getConnecteePaths(input) == expectedConnecteePaths);
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

TEST_CASE("Component Interface Single Value Input Connectee Serialization Behavior")
{
    // Test normal behavior of single-value input (de)serialization.
    // -------------------------------------------------------------

    writeTimeSeriesTableForInputConnecteeSerialization();

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
        SimTK_TEST(!foo->updInput("input1").isListSocket());
        SimTK_TEST(!foo->updInput("fiberLength").isListSocket());

        // Add to world.
        world.add(source);
        world.add(foo);

        // Connect, finalize, etc.
        const auto& output = source->getOutput("column");
        // See if we preserve the ordering of the channels.
        foo->connectInput_input1(output.getChannel("b"));
        // We want to make sure aliases are preserved.
        foo->connectInput_fiberLength(output.getChannel("d"), "desert");
        // Ensure that re-finalizing from properties does not cause Inputs
        // to hold onto stale references to the outputs' channels.
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

        // We won't wire up this input, but its connectee path should still
        // (de)serialize.
        foo->updInput("activation").setConnecteePath("non/existent");

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
        SimTK_TEST(!input1.isListSocket());
        SimTK_TEST(!fiberLength.isListSocket());
        SimTK_TEST(!activation.isListSocket());

        // Check connectee paths before *and* after connecting, since
        // the connecting process edits the connectee_name property.
        SimTK_TEST(input1.getConnecteePath() == "/producer|column:b");
        SimTK_TEST(fiberLength.getConnecteePath() ==
                   "/producer|column:d(desert)");
        // Even if we hadn't wired this up, its name still deserializes:
        SimTK_TEST(activation.getConnecteePath() == "non/existent");
        // Now we must clear this before trying to connect, since the connectee
        // doesn't exist.
        activation.setConnecteePath("");

        // Connect.
        world.connect();

        // Make sure these inputs are single-value even after connecting.
        SimTK_TEST(!input1.isListSocket());
        SimTK_TEST(!fiberLength.isListSocket());
        SimTK_TEST(!activation.isListSocket());

        SimTK_TEST(input1.getConnecteePath() == "/producer|column:b");
        SimTK_TEST(fiberLength.getConnecteePath() ==
                   "/producer|column:d(desert)");

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
        // connectee paths for a single-value input.
        auto& connectee_name = Property<std::string>::updAs(
                        foo->updPropertyByName("input_input1"));
        connectee_name.setAllowableListSize(0, 10);
        connectee_name.appendValue("apple");
        connectee_name.appendValue("banana");
        connectee_name.appendValue("lemon");

        world.print(modelFileNameMultipleValues);
    }
    // Deserialize.
    {
        // Single-value connectee cannot have multiple connectee_names.
        SimTK_TEST_MUST_THROW_EXC(
             TheWorld world(modelFileNameMultipleValues),
             OpenSim::Exception);
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
        input1.setConnecteePath("abc+def"); // '+' is invalid for ComponentPath.
        // The check for invalid names occurs in
        // AbstractSocket::checkConnecteePathProperty(), which is invoked
        // by the following function:
        SimTK_TEST_MUST_THROW_EXC(foo->finalizeFromProperties(),
                                  OpenSim::Exception);
        world.print(modelFileNameInvalidChar);
    }
    // Deserialize.
    {
        // Make sure that deserializing a Component with an invalid
        // connectee_name throws an exception.
        SimTK_TEST_MUST_THROW_EXC(TheWorld world(modelFileNameInvalidChar),
                                  OpenSim::Exception);
    }
}

TEST_CASE("Component Interface Aliases and Labels Behavior")
{
    auto theWorld = std::unique_ptr<TheWorld>(new TheWorld());
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
    foo->connectInput_input1( bar->getOutput("Output1") );
    theWorld->connect();
    SimTK_TEST(foo->getInput("input1").getAlias().empty());
    SimTK_TEST(foo->getInput("input1").getLabel() == "/bar|Output1");

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
    foo->connectInput_input1( bar->getOutput("Output1"), "baz" );
    theWorld->connect();
    SimTK_TEST(foo->getInput("input1").getAlias() == "baz");
    SimTK_TEST(foo->getInput("input1").getLabel() == "baz");

    // List Input, no aliases.
    foo->connectInput_listInput1( bar->getOutput("Output1") );
    foo->connectInput_listInput1( bar->getOutput("Output3") );
    theWorld->connect();

    ASSERT_THROW(OpenSim::Exception, foo->getInput("listInput1").getAlias());
    ASSERT_THROW(OpenSim::Exception, foo->getInput("listInput1").getLabel());

    SimTK_TEST(foo->getInput("listInput1").getAlias(0).empty());
    SimTK_TEST(foo->getInput("listInput1").getLabel(0) == "/bar|Output1");

    SimTK_TEST(foo->getInput("listInput1").getAlias(1).empty());
    SimTK_TEST(foo->getInput("listInput1").getLabel(1) == "/bar|Output3");

    foo->updInput("listInput1").disconnect();

    // List Input, with aliases.
    foo->connectInput_listInput1( bar->getOutput("Output1"), "plugh" );
    foo->connectInput_listInput1( bar->getOutput("Output3"), "thud" );
    theWorld->connect();

    SimTK_TEST(foo->getInput("listInput1").getAlias(0) == "plugh");
    SimTK_TEST(foo->getInput("listInput1").getLabel(0) == "plugh");

    SimTK_TEST(foo->getInput("listInput1").getAlias(1) == "thud");
    SimTK_TEST(foo->getInput("listInput1").getLabel(1) == "thud");
}

TEST_CASE("Component Interface Get Absolute Path String Speed")
{
    std::clock_t constructStartTime = std::clock();

    TheWorld* A = new TheWorld();
    TheWorld* B = new TheWorld();
    TheWorld* C = new TheWorld();
    TheWorld* D = new TheWorld();
    TheWorld* E = new TheWorld();
    // Use longer names to avoid short string optimization
    A->setName("a2345678901234567890");
    B->setName("b2345678901234567890");
    C->setName("c2345678901234567890");
    D->setName("d2345678901234567890");
    E->setName("e2345678901234567890");

    A->add(B);
    B->add(C);
    C->add(D);
    D->add(E);

    double avgTime = 0;
    int numTrials = 10;
    int numLoops = 1000000;
    for (int trial = 0; trial < numTrials; ++trial) {
        std::clock_t loopStartTime = std::clock();
        for (int i = 0; i < numLoops; ++i) {
            A->getAbsolutePathString();
            B->getAbsolutePathString();
            C->getAbsolutePathString();
            D->getAbsolutePathString();
            E->getAbsolutePathString();
        }
        std::clock_t loopEndTime = std::clock();
        double loopClocks = loopEndTime - loopStartTime;
        avgTime += loopClocks / CLOCKS_PER_SEC;
    }

    cout << "getAbsolutePathString avgTime = " << avgTime / numTrials << "s" << endl;

    avgTime = 0;
    for (int trial = 0; trial < numTrials; ++trial) {
        std::clock_t loopStartTime = std::clock();
        for (int i = 0; i < numLoops; ++i) {
            A->getName();
            B->getName();
            C->getName();
            D->getName();
            E->getName();
        }
        std::clock_t loopEndTime = std::clock();
        double loopClocks = loopEndTime - loopStartTime;
        avgTime += loopClocks / CLOCKS_PER_SEC;
    }

    cout << "getName avgTime = " << avgTime / numTrials << "s" << endl;
}

TEST_CASE("Component Interface Formatted DateTime Works as Expected")
{
    std::string withMicroseconds = getFormattedDateTime(true, "%Y");
    std::string withoutMicroseconds = getFormattedDateTime(false, "%Y");
    SimTK_TEST(withMicroseconds.find(withoutMicroseconds) == 0);
}

struct ComponentWithCacheVariable : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(ComponentWithCacheVariable, Component);
public:
    CacheVariable<double> cv;

    using Component::addCacheVariable;

};

template<class T>
using CacheVariable = Component::CacheVariable<T>;

TEST_CASE("Component Interface CacheVariable<T> Behavior")
{
    // can default-initialize without throwing an exception
    {
        CacheVariable<double> cv;
    }

    // can value-initialize without throwing an exception
    {
        CacheVariable<double> cv{};
    }

    // can be rvalue-initialized via `Component::addCacheVariable`
    // without throwing an exception.
    {
        ComponentWithCacheVariable c{};
        CacheVariable<double> cv{
            c.addCacheVariable("name", 0.0, SimTK::Stage::Velocity)
        };
    }

    // can be rvalue-assigned via Component::addCacheVariable without exception
    {
        ComponentWithCacheVariable c{};
        CacheVariable<double> cv =
                c.addCacheVariable("name", 0.0, SimTK::Stage::Velocity);
    }

    // can be copy-initialized without exception
    {
        ComponentWithCacheVariable c{};
        CacheVariable<double> cv =
                c.addCacheVariable("name", 0.0, SimTK::Stage::Velocity);

        CacheVariable<double> cv2{cv};
    }

    // can be copy-assigned without exception
    {
        ComponentWithCacheVariable c{};
        CacheVariable<double> cv = c.addCacheVariable("name", 0.0, SimTK::Stage::Velocity);
        CacheVariable<double> cv2 = cv;
    }

    // the Component's cache variable value methods (e.g. Component::getCacheVariableValue)
    // should throw if `Component::realizeTopology` has not yet been called
    {
        ComponentWithCacheVariable c{};
        const std::string k = "name";
        double v = 1337.0;
        CacheVariable<double> cv = c.addCacheVariable("name", v, SimTK::Stage::Time);
        c.finalizeFromProperties();

        SimTK::State s;  // note: no initialization of the component

        ASSERT_THROW(std::exception, c.getCacheVariableIndex(k));
        ASSERT_THROW(std::exception, c.getCacheVariableIndex(cv));
        ASSERT_THROW(std::exception, c.getCacheVariableValue<double>(s, k));
        ASSERT_THROW(std::exception, c.getCacheVariableValue(s, cv));
        ASSERT_THROW(std::exception, c.setCacheVariableValue<double>(s, k, v));
        ASSERT_THROW(std::exception, c.setCacheVariableValue(s, k, cv));
        ASSERT_THROW(std::exception, c.updCacheVariableValue<double>(s, k));
        ASSERT_THROW(std::exception, c.updCacheVariableValue(s, cv));
        ASSERT_THROW(std::exception, c.updCacheVariableValue(s, cv));
        ASSERT_THROW(std::exception, c.isCacheVariableValid(s, k));
        ASSERT_THROW(std::exception, c.isCacheVariableValid(s, cv));
        ASSERT_THROW(std::exception, c.markCacheVariableValid(s, k));
        ASSERT_THROW(std::exception, c.markCacheVariableValid(s, cv));
        ASSERT_THROW(std::exception, c.markCacheVariableInvalid(s, k));
        ASSERT_THROW(std::exception, c.markCacheVariableInvalid(s, cv));
    }

    // C++ rng for doubles for these tests
    std::uniform_real_distribution<double> dist{};
    std::default_random_engine engine(static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count()));
    auto generate_random_double = [&]() {
        return dist(engine);
    };

    // the component's cache variable value methods (e.g. Component::getCacheVariableValue)
    // should work after `Component::realizeTopology` has been called
    {
        SimTK::MultibodySystem sys;
        SimbodyMatterSubsystem matter{sys};
        GeneralForceSubsystem forces{sys};

        ComponentWithCacheVariable c{};
        c.finalizeFromProperties();
        c.finalizeConnections(c);
        c.addToSystem(sys);

        const std::string k = "name";
        const double v = generate_random_double();
        c.cv = c.addCacheVariable(k, v, SimTK::Stage::Model);

        SimTK::State s = sys.realizeTopology();  // note: this initializes the value in SimTK

        c.markCacheVariableValid(s, k);
        ASSERT(c.isCacheVariableValid(s, c.cv) == true);
        c.markCacheVariableInvalid(s, k);
        ASSERT(c.isCacheVariableValid(s, c.cv) == false);

        c.markCacheVariableValid(s, c.cv);
        ASSERT(c.isCacheVariableValid(s, c.cv) == true);
        c.markCacheVariableInvalid(s, c.cv);
        ASSERT(c.isCacheVariableValid(s, c.cv) == false);

        // tests assume cv is valid from now on
        c.markCacheVariableValid(s, c.cv);

        c.getCacheVariableIndex(k);  // shouldn't throw
        ASSERT(c.getCacheVariableValue<double>(s, k) == v);
        ASSERT(c.getCacheVariableValue(s, c.cv) == v);

        // Setting the value via key causes subsequent `get` methods
        // (both string key + CacheVariable ones) to return the new value.
        {
            double v2 = generate_random_double();
            c.setCacheVariableValue<double>(s, k, v2);
            ASSERT(c.getCacheVariableValue<double>(s, k) == v2);
            ASSERT(c.getCacheVariableValue(s, c.cv) == v2);
        }

        // Same as above, but setting via a CacheVariable<T>.
        {
            double v2 = generate_random_double();
            c.setCacheVariableValue<double>(s, c.cv, v2);
            ASSERT(c.getCacheVariableValue<double>(s, k) == v2);
            ASSERT(c.getCacheVariableValue(s, c.cv) == v2);
        }

        // Updating a value via an `upd` reference, retrieved via a key, causes
        // subsequent `get` methods (both string key + CacheVariable<T> ones)
        // to return the updated value
        {
            double v2 = generate_random_double();
            double& v = c.updCacheVariableValue<double>(s, k);
            v = v2;
            ASSERT(c.getCacheVariableValue<double>(s, k) == v2);
            ASSERT(c.getCacheVariableValue(s, c.cv) == v2);
        }

        // Same as above, but getting the `upd` reference via a CacheVariable<T>
        {
            double v2 = generate_random_double();
            double& v = c.updCacheVariableValue(s, c.cv);
            v = v2;
            ASSERT(c.getCacheVariableValue<double>(s, k) == v2);
            ASSERT(c.getCacheVariableValue(s, c.cv) == v2);
        }
    }

    // when a CacheVariable is copy-constructed from an already-initialized CacheVariable,
    // the copy cannot get/update cache variable values from the original. The variable
    // must be reinitialized (with `Component::addCacheVariable` followed by
    // `Component::realizeTopology`)
    {
        SimTK::MultibodySystem sys;
        SimbodyMatterSubsystem matter{sys};
        GeneralForceSubsystem forces{sys};

        ComponentWithCacheVariable c1{};
        c1.finalizeFromProperties();
        c1.finalizeConnections(c1);
        c1.addToSystem(sys);

        const std::string k = "name";
        const double v = generate_random_double();
        c1.cv = c1.addCacheVariable(k, v, SimTK::Stage::Model);

        SimTK::State s = sys.realizeTopology();

        // Normal usage with no copies works as expected
        c1.markCacheVariableValid(s, c1.cv);
        ASSERT(c1.getCacheVariableValue(s, c1.cv) == v);

        // create a copy via standard C++ semantics
        ComponentWithCacheVariable c2 = c1;

        // c2.cv is only copy-initialized, it must be allocated with
        // `Component::addCacheVariable`
        ASSERT_THROW(std::exception, c2.getCacheVariableValue(s, c2.cv));

        // calling `Component::addToSystem` usually allocates the cache variable
        // in `Component` (in standard implementations, `extendAddToSystem`
        // handles that). Here, the allocation step is performed explicitly.
        c2.finalizeFromProperties();
        c2.finalizeConnections(c2);
        c2.addToSystem(sys);

        // throws, because `ComponentWithCacheVariable` does not actually
        // declare a `extendAddToSystem` in which `addCacheVariable` is called,
        // so the cache variable is still in the copy-initialized state.
        ASSERT_THROW(std::exception, c2.getCacheVariableValue(s, c2.cv));

        c2.addCacheVariable(k, v, SimTK::Stage::Model);

        // throws because `Component::addCacheVariable` only allocates the
        // variable in the `Component`, not SimTK.
        ASSERT_THROW(std::exception, c2.getCacheVariableValue(s, c2.cv));

        // calling `SimTK::System::realizeTopology` is what actually initializes
        // the value in a *newly created* `SimTK::State`, so this does not throw
        SimTK::State s2 = sys.realizeTopology();
        c2.markCacheVariableValid(s2, c2.cv);

        // works without throwing
        ASSERT(c2.getCacheVariableValue(s2, c2.cv) == v);
    }

    // Component::addCacheVariable cannot be called with the same name twice because
    // doing so could cause confusing run-time errors. Therefore, when callers (wrongly)
    // provide a duplicate key, the implementation should fail fast and tell the
    // developer that duplicate names are not supported.
    {
        ComponentWithCacheVariable c{};
        const std::string key = "duplicateKey";
        double v1 = 1.0;
        unsigned v2 = 2;  // it doesn't matter if the values/types differ: they still have the same key
        CacheVariable<double> cv1 = c.addCacheVariable(key, v1, SimTK::Stage::Model);
        ASSERT_THROW(std::exception, c.addCacheVariable(key, v2, SimTK::Stage::Model));
    }
}

// repro related to #3409/#3411
//
// if downstream code tries to read an optional SimpleProperty without
// checking whether it's populated or not, it should throw an exception
// rather than segfaulting
TEST_CASE("Component Interface: Incorrectly Reading An Optional Simple Property Should Not Segfault (#3409/#3411)")
{
    ComponentWithOptionalSimpleProperty c;

    ASSERT_EQUAL(c.getProperty_num().size(), 0);
    try {
        // shouldn't segfault...
        ASSERT_EQUAL(c.get_num(), 1337);
    } catch (const std::exception&) {
        // ... but should throw an exception
    }
}

// repro related to #3347
//
// if downstream code (e.g. OpenSim::CoordinateCouplerConstraint in
// OpenSim <=4.4) tries to read an optional ObjectProperty without checking
// whether it's populated or not, it should throw an exception rather
// than segfaulting
TEST_CASE("Component Interface: Incorrectly Reading an Optional Object Property Should Not Segfault (#3347)")
{
    ComponentWithOptionalObjectProperty c;

    ASSERT_EQUAL(c.getProperty_func().size(), 0);
    try {
        // shouldn't segfault...
        c.get_func().getName();
    } catch (const std::exception&) {
        // ... but should throw an exception
    }
}

TEST_CASE("Component Interface: removeComponent returns `true` if it removes a component added via addComponent")
{
    BlankComponent c;
    BlankComponent* child = new BlankComponent{};
    c.addComponent(child);
    CHECK(c.removeComponent(child));
}

TEST_CASE("Component Interface: removeComponent returns `false` if given a `Component` that wasn't added via addComponent")
{
    BlankComponent c;
    auto unrelated = std::make_unique<BlankComponent>();
    CHECK(!c.removeComponent(unrelated.get()));
}

TEST_CASE("Component Interface: removeComponent returns `false` if caller tries to remove a direct object property")
{
    ComponentWithObjectProperty c;
    Component* property = &c.upd_object_property();
    CHECK(!c.removeComponent(property));
}

TEST_CASE("Component Interface: extractComponent returns the component added via addComponent")
{
    BlankComponent c;
    BlankComponent* child = new BlankComponent{};
    c.addComponent(child);
    auto extracted = c.extractComponent(child);
    CHECK(extracted.get() == child);
}

TEST_CASE("Component Interface: extractComponent returns nullptr if given a `Component` that wasn't added via addComponent")
{
    BlankComponent c;
    auto unrelated = std::make_unique<BlankComponent>();
    CHECK(c.extractComponent(unrelated.get()) == nullptr);
}

TEST_CASE("Component Interface: extractComponent returns `nullptr` if caller tries to remove a direct object property")
{
    ComponentWithObjectProperty c;
    Component* property = &c.upd_object_property();
    CHECK(c.extractComponent(property).get() == nullptr);
}

const bool g_TestFixtureTypesAreRegistered = []()
{
    // ensure new types are globally registered for testing deserialization
    Object::registerType(Foo());
    Object::registerType(Bar());
    Object::registerType(TheWorld());
    return true;
}();
