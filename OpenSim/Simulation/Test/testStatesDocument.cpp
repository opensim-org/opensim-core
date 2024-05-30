/* -------------------------------------------------------------------------- *
*                OpenSim:  testComponentInterface.cpp                        *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2024 Stanford University and the Authors                     *
* Author(s): F. C. Anderson                                                  *
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

//-----------------------------------------------------------------------------
// The section above was taken from testComponentInterface.cpp.
// The code is adapted to generate a dummy model with a SimTK::State that is
// comprised of continuous variables (q, u, z), discrete variables of all
// supported types, and modeling options.
//-----------------------------------------------------------------------------
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

        // Modeling Option Initialization
        std::string moName = "moY";
        int moVal{0};
        SimTK::DiscreteVariableIndex moIndex =
            fsub.allocateDiscreteVariable(state, Stage::Dynamics,
                new Value<int>(moVal));
        initializeModelingOptionIndexes(moName,
            fsub.getMySubsystemIndex(), moIndex);
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

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// The section above was taken from testComponentInterface.cpp.
// The code is adapted to generate a dummy model with a SimTK::State that is
// comprised of continuous variables (q, u, z), discrete variables of all
// supported types, and modeling options.
//-----------------------------------------------------------------------------



// Internal static methods and classes.
namespace
{
    double customSquare(double x)
    {
        return(x*x);
    }
}



TEST_CASE("Getting Started")
{
    double x = 2.0;
    double square = customSquare(x);
    REQUIRE(square == x*x);
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
