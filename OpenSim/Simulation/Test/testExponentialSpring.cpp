/* -------------------------------------------------------------------------- *
 *               OpenSim:  testContactExponentialSpring.cpp                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2022 Stanford University and the Authors                     *
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

#include <iostream>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/Array.h>

#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/ForceReporter.h>

#include <OpenSim/Simulation/Model/ContactGeometrySet.h>
#include <OpenSim/Simulation/Model/ContactHalfSpace.h>
#include <OpenSim/Simulation/Model/ContactMesh.h>
#include <OpenSim/Simulation/Model/ContactSphere.h>
#include <OpenSim/Simulation/Model/ElasticFoundationForce.h>
#include <OpenSim/Simulation/Model/HuntCrossleyForce.h>
#include <OpenSim/Simulation/Model/ExponentialSpringForce.h>
#include <OpenSim/Simulation/Model/ExternalForce.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include "SimTKsimbody.h"

using namespace SimTK;
using namespace std;
using namespace OpenSim;

class TestExpSprVisuals;

//=============================================================================
/** Record the System state at regular intervals. */
class VisualsReporter : public PeriodicEventReporter
{
public:
    VisualsReporter(TestExpSprVisuals& visuals, Real reportInterval) :
        PeriodicEventReporter(reportInterval), vis(visuals) {}

    void handleEvent(const State& state) const override;

private:
    TestExpSprVisuals& vis;
};

//=============================================================================
/** Class Visuals encapsulates the variables needed for running a
SimTK::Visualizer and provides the polling methods for starting a
simulation and replaying the simulated motion. */
class TestExpSprVisuals
{
public:
    // Constructor
    TestExpSprVisuals(MultibodySystem& system) {
        // Menu Items
        runMenuItems.push_back(std::make_pair("Go", goItem));
        runMenuItems.push_back(std::make_pair("Replay", replayItem));

        // SimTK::Visualizer
        vis = new Visualizer(system);
        vis->setShowShadows(true);

        // Input Silo
        silo = new Visualizer::InputSilo();
        vis->addInputListener(silo);
        vis->addMenu("Run", runMenuID, runMenuItems);

        // Reporters
        system.addEventReporter(new Visualizer::Reporter(*vis, reportInterval));
        system.addEventReporter(new VisualsReporter(*this, reportInterval));

        // Reserve memory for the System states to be recorded
        states.reserve(50000);
    }

    // State storage
    void appendState(const State& state) { states.emplace_back(state); }

    // Polling
    void pollForStart() {
        cout << "\nChoose 'Go' from the Run menu to simulate.\n";
        int menuID, item;
        do {
            silo->waitForMenuPick(menuID, item);
            if (menuID != runMenuID || item != goItem)
                cout << "\aDude ... follow instructions!\n";
        } while (menuID != runMenuID || item != goItem);
    }
    void pollForReplay() {
        silo->clear();
        while (true) {
            cout << "Choose Replay to see that again ...\n";
            int menuID, item;
            silo->waitForMenuPick(menuID, item);
            for (double i = 0; i < (int)states.size(); i++) {
                vis->report(states[(int)i]);
            }
        }
    }

private:
    SimTK::Visualizer* vis{NULL};
    SimTK::Visualizer::InputSilo* silo{NULL};
    SimTK::Array_<std::pair<String, int>> runMenuItems;
    const int runMenuID = 3;
    const int goItem{1}, replayItem{2}, quitItem{3}; 
    double reportInterval = 0.01;
    SimTK::Array_<State> states;
};

//_____________________________________________________________________________
// Definition had to follow the declaration for TestExpSprVisuals
void
VisualsReporter::
handleEvent(const State& state) const
{
    vis.appendState(state);
}

//=============================================================================
/** Class ExponentialSpringTester provides a scope and framework for
evaluating and testing the ExponentialSpringForce class. Using a class gets
a lot of variables out of the global scope. */
class ExponentialSpringTester
{
public:
    // Contact choices
    enum ContactChoice {
        ExpSpr = 0,
        HuntCross,
        Both
    };

    // Initial condition choices
    enum InitialConditionsChoice{
        Static = 0,
        Bounce,
        Slide,
        Spin,
        Tumble
    };

    // Constructor
    ExponentialSpringTester() {};

    // Destructor
    ~ExponentialSpringTester() {
        if(model) model->disownAllComponents();
        if(model) delete model;
    }

    // Command line parsing and usage
    int parseCommandLine(int argc, char** argv);
    void printUsage();

    // Model Creation
    void buildModel();
    OpenSim::Body* addBlock(const std::string& suffix);
    void addExponentialSprings(OpenSim::Body* body);
    void addHuntCrossleyContact(OpenSim::Body* body);
    void setForceData(
            double t, const SimTK::Vec3& point, const SimTK::Vec3& force);
    void setForceDataHeader();
    void addDecorations();

    // Simulation
    void setInitialConditions(SimTK::State& state,
        const SimTK::MobilizedBody& body, double dz);
    void simulate();

    //-------------------------------------------------------------------------
    // Member variables
    //-------------------------------------------------------------------------
 private:

    // Simulation related
    double integ_accuracy{1.0e-5};
    SimTK::Vec3 gravity{SimTK::Vec3(0, -9.8065, 0)};
    double mass{10.0};
    double hs{0.10};  // half of a side of a cube (kind of like a radius)
    double tf{5.0};

    // Command line options and their defaults
    ContactChoice whichContact{ExpSpr};
    InitialConditionsChoice whichInit{Slide};
    bool noDamp{false};
    bool applyFx{false};
    bool showVisuals{false};

    // Model and parts
    Model* model{NULL};
    OpenSim::Body* blockES{NULL};
    OpenSim::Body* blockHC{NULL};
    Storage fxData;
    ExternalForce* fxES{NULL};
    ExternalForce* fxHC{NULL};

    // Visualization
    TestExpSprVisuals* visuals{NULL};

}; // End class ExponentialSpringTester declarations

//_____________________________________________________________________________
void
ExponentialSpringTester::
addDecorations()
{
    // Ground
    Ground& ground = model->updGround();
    SimTK::DecorativeBrick* floor =
            new SimTK::DecorativeBrick(Vec3(2.0, 0.5, 2.0));
    floor->setColor(Green);
    floor->setOpacity(0.1);
    SimTK::Body& grndBody = ground.updMobilizedBody().updBody();
    grndBody.addDecoration(Transform(Vec3(0, -0.5, 0)), *floor);

    // Exponential Springs Block
    if (blockES) {
        SimTK::Body& body = blockES->updMobilizedBody().updBody();
        SimTK::DecorativeBrick* brick =
                new SimTK::DecorativeBrick(SimTK::Vec3(hs));
        brick->setColor(SimTK::Blue);
        body.addDecoration(SimTK::Transform(), *brick);
    }

    // Hunt-Crossley Block
    if (blockHC) {
        SimTK::Body& body = blockHC->updMobilizedBody().updBody();
        SimTK::DecorativeBrick* brick =
                new SimTK::DecorativeBrick(SimTK::Vec3(hs));
        brick->setColor(SimTK::Red);
        body.addDecoration(SimTK::Transform(), *brick);
    }
}
//_____________________________________________________________________________
int
ExponentialSpringTester::
parseCommandLine(int argc, char** argv)
{
    std::string option;
    for (int i = 1; i < argc; ++i) {

        option = argv[i];

        // Contact choice
        if (option == "ExpSpr")
            whichContact = ExpSpr;
        else if (option == "HuntCross")
            whichContact = HuntCross;
        else if (option == "Both")
            whichContact = Both;

        // Initial condition choice
        else if (option == "Static")
            whichInit = Static;
        else if (option == "Bounce")
            whichInit = Bounce;
        else if (option == "Slide")
            whichInit = Slide;
        else if (option == "Spin")
            whichInit = Spin;
        else if (option == "Tumble")
            whichInit = Tumble;

        // Turn off all dissipative terms
        else if (option == "NoDamp")
            noDamp = true;

        // Apply a horizontal ramping force
        else if (option == "Fx")
            applyFx = true;

        // Show the visuals
        else if (option == "Vis")
            showVisuals = true;

        // Unrecognized
        else {
            printUsage();
            return -1;
        }
    }
    return 0;
}
//_____________________________________________________________________________
void
ExponentialSpringTester::
printUsage()
{
    cout << endl << "Usage:" << endl;
    cout << "$ testExponetialSpring "
         << "[InitCond] [Contact] [NoDamp] [ApplyFx] [Vis]" << endl;
    cout << "\tInitCond (choose one): Static Bounce Slide Spin Tumble ";
    cout << endl;
    cout << "\t Contact (choose one): ExpSpr HuntCross Both" << endl << endl;

    cout << "All arguments are optional. If no arguments are specified, ";
    cout << "a 'Slide' will" << endl;
    cout << "be simulated with one block that uses ";
    cout << "ExponentialSpringForce contact," << endl;
    cout << "with typical damping settings, ";
    cout << "with no extnerally applied force, " << endl;
    cout << "and with no visuals." << endl << endl;

    cout << "Example:" << endl;
    cout << "To simulated 2 blocks (one with Exponential Springs and one";
    cout << " with Hunt-Crossley)" << endl;
    cout << "that bounce without energy dissipation and with Visuals, ";
    cout << "enter the following: " << endl << endl;

    cout << "$ testExponentialSpring Bounce Both NoDamp Vis" << endl << endl;
}
//_____________________________________________________________________________
// Build the model
void
ExponentialSpringTester::buildModel()
{
    // Create the bodies
    model = new Model();
    model->setGravity(gravity);
    model->setName("TestExponentialSpring");
    switch (whichContact) {
    case ExpSpr:
        blockES = addBlock("ES");
        addExponentialSprings(blockES);
        break;
    case HuntCross:
        blockHC = addBlock("HC");
        addHuntCrossleyContact(blockHC);
        break;
    case Both:
        blockES = addBlock("ES");
        addExponentialSprings(blockES);
        blockHC = addBlock("HC");
        addHuntCrossleyContact(blockHC);
    }

    // Add the external force
    if (applyFx) {
        setForceDataHeader();
        SimTK::Vec3 point(0.0, -hs, 0.0);
        SimTK::Vec3 zero(0.0);
        SimTK::Vec3 force(-0.7*gravity[1]*mass, 0.0, 0.0);
        setForceData(0.0, point, zero);
        setForceData(tf, point, zero);
        tf = tf + 25.0;
        setForceData(tf, point, force);
        if (blockES) {
            cout << "Adding fx for " << blockES->getName() << endl;
            fxData.print("C:\\Users\\fcand\\Documents\\fxData.sto");
            fxES = new ExternalForce(fxData,"force", "point", "",
                blockES->getName(), "ground", blockES->getName());
            fxES->setName("externalforceES");
            model->addForce(fxES);
        }
        if (blockHC) {
            cout << "Adding fx for " << blockHC->getName() << endl;
            fxData.print("C:\\Users\\fcand\\Documents\\fxData.sto");
            fxHC = new ExternalForce(fxData, "force", "point", "",
                blockHC->getName(), "ground", blockHC->getName());
            fxHC->setName("externalforceHC");
            model->addForce(fxHC);
        }
    }
}
//______________________________________________________________________________
void ExponentialSpringTester::setForceDataHeader()
{
    fxData.setName("fx");
    fxData.setDescription("An external force applied to a block.");
    Array<std::string> lab; // labels
    lab.append("time");
    lab.append("point.x");
    lab.append("point.y");
    lab.append("point.z");
    lab.append("force.x");
    lab.append("force.y");
    lab.append("force.z");
    fxData.setColumnLabels(lab);
}

//______________________________________________________________________________
void
ExponentialSpringTester::
setForceData(double t, const SimTK::Vec3& point, const SimTK::Vec3& force)
{
    SimTK::Vector_<double> data(6);
    for (int i = 0; i < 3; ++i) {
        data[i] = point[i];
        data[3 + i] = force[i];
    }
    StateVector sv(t, data);
    fxData.append(sv);
}

//______________________________________________________________________________
OpenSim::Body*
ExponentialSpringTester::
addBlock(const std::string& suffix)
{
    Ground& ground = model->updGround();

    // Body
    std::string name = "block" + suffix;
    OpenSim::Body* block = new OpenSim::Body();
    block->setName(name);
    block->set_mass(mass);
    block->set_mass_center(Vec3(0));
    block->setInertia(Inertia(1.0));

    // Joint
    name = "free" + suffix;
    FreeJoint *free = new
        FreeJoint(name, ground, Vec3(0), Vec3(0), *block, Vec3(0), Vec3(0));
    model->addBody(block);
    model->addJoint(free);

    return block;
}
//______________________________________________________________________________
void
ExponentialSpringTester
::addExponentialSprings(OpenSim::Body* block)
{
    Ground& ground = model->updGround();

    // Corners of the block
    const int n = 8;
    Vec3 corner[n];
    corner[0] = Vec3(hs, -hs, hs);
    corner[1] = Vec3(hs, -hs, -hs);
    corner[2] = Vec3(-hs, -hs, -hs);
    corner[3] = Vec3(-hs, -hs, hs);
    corner[4] = Vec3(hs, hs, hs);
    corner[5] = Vec3(hs, hs, -hs);
    corner[6] = Vec3(-hs, hs, -hs);
    corner[7] = Vec3(-hs, hs, hs);

    // Contact Plane Transform
    Real angle = convertDegreesToRadians(90.0);
    Rotation floorRot(-angle, XAxis);
    Vec3 floorOrigin(0., -0.004, 0.);
    Transform floorXForm(floorRot, floorOrigin);

    // Specify non-default contact parameters
    SimTK::ExponentialSpringParameters params;
    if (noDamp) {
        params.setNormalViscosity(0.0);
        params.setFrictionViscosity(0.0);
        params.setInitialMuStatic(0.0);
    }

    // Loop over the corners
    std::string name = "";
    for (int i = 0; i < n; ++i) {
        name = "ExpSpr" + std::to_string(i);
        OpenSim::ExponentialContact* force =
            new OpenSim::ExponentialContact(floorXForm,
                block->getName(), corner[i], params);
        force->setName(name);
        model->addForce(force);
    }
}
//______________________________________________________________________________
void
ExponentialSpringTester::
addHuntCrossleyContact(OpenSim::Body* block)
{
    Ground& ground = model->updGround();

    // Geometry for the floor
    ContactHalfSpace* floor = new ContactHalfSpace(
            Vec3(0), Vec3(0, 0, -0.5 * SimTK_PI), ground, "floor");
    model->addContactGeometry(floor);

    // Corners of the block
    const int n = 8;
    Vec3 corner[n];
    corner[0] = Vec3(hs, -hs, hs);
    corner[1] = Vec3(hs, -hs, -hs);
    corner[2] = Vec3(-hs, -hs, -hs);
    corner[3] = Vec3(-hs, -hs, hs);
    corner[4] = Vec3(hs, hs, hs);
    corner[5] = Vec3(hs, hs, -hs);
    corner[6] = Vec3(-hs, hs, -hs);
    corner[7] = Vec3(-hs, hs, hs);

    // Loop over the corners
    std::string name = "";
    for (int i = 0; i < n; ++i) {
        // Geometry
        name = "sphere_" + std::to_string(i); 
        OpenSim::ContactGeometry* geometry =
            new ContactSphere(0.005, corner[i], *block, name);
        model->addContactGeometry(geometry);

        // HuntCrossleyForce
        auto* contactParams = new OpenSim::HuntCrossleyForce::
            ContactParameters(1.0e7, 4e-1, 0.7, 0.465, 0.0);
        contactParams->addGeometry(name);
        contactParams->addGeometry("floor");
        OpenSim::HuntCrossleyForce* force =
            new OpenSim::HuntCrossleyForce(contactParams);
        name = "HuntCrossleyForce_" + std::to_string(i);
        force->setName(name);
        force->setTransitionVelocity(0.01);
        model->addForce(force);
    }
}
//_____________________________________________________________________________
void
ExponentialSpringTester::
setInitialConditions(SimTK::State& state, const SimTK::MobilizedBody& body,
    double dz)
{
    SimTK::Vec3 pos(0.0, 0.0, dz);
    SimTK::Vec3 vel(0.0);
    SimTK::Vec3 angvel(0.0);

    switch (whichInit) {
    case Static:
        pos[0] = 0.0;
        pos[1] = hs;
        body.setQToFitTranslation(state, pos);
        break;
    case Bounce:
        pos[0] = 0.0;
        pos[1] = 1.0;
        body.setQToFitTranslation(state, pos);
        break;
    case Slide:
        pos[0] = 2.0;
        pos[1] = 2.0 * hs;
        vel[0] = -4.0;
        body.setQToFitTranslation(state, pos);
        body.setUToFitLinearVelocity(state, vel);
        break;
    case Spin:
        pos[0] = 0.0;
        pos[1] = hs;
        vel[0] = 0.0;
        angvel[1] = 4.0 * SimTK::Pi;
        body.setQToFitTranslation(state, pos);
        body.setUToFitLinearVelocity(state, vel);
        body.setUToFitAngularVelocity(state, angvel);
        break;
    case Tumble:
        pos[0] = -1.5;
        pos[1] = 2.0 * hs;
        vel[0] = -1.0;
        angvel[2] = 2.0 * SimTK::Pi;
        body.setQToFitTranslation(state, pos);
        body.setUToFitLinearVelocity(state, vel);
        body.setUToFitAngularVelocity(state, angvel);
        break;
    default:
        cout << "Unrecognized set of initial conditions!" << endl;
    }

}
//_____________________________________________________________________________
void
ExponentialSpringTester::
simulate()
{
    // Visuals?
    if (showVisuals) {
        if (blockES) {
            auto blockESGeometry = new Brick(Vec3(hs));
            blockESGeometry->setColor(Vec3(0.1, 0.1, 0.8));
            blockES->attachGeometry(blockESGeometry);
        }
        if (blockHC) {
            auto blockHCGeometry = new Brick(Vec3(hs));
            blockHCGeometry->setColor(Vec3(0.8, 0.1, 0.1));
            blockHC->attachGeometry(blockHCGeometry);
        }
        model->setUseVisualizer(true);
    }

    // Initialize the System
    SimTK::State& state = model->initSystem();

    // Set initial conditions
    double dz = 1.0;
    if (blockES != NULL)
        setInitialConditions(state, blockES->getMobilizedBody(), dz);
    if (blockHC != NULL)
        setInitialConditions(state, blockHC->getMobilizedBody(), -dz);

    // Integrate
    Manager manager(*model);
    manager.getIntegrator().setMaximumStepSize(0.02);
    manager.setIntegratorAccuracy(integ_accuracy);
    state.setTime(0.0);
    manager.initialize(state);
    std::clock_t startTime = std::clock();
    state = manager.integrate(tf);
    auto runTime = 1.e3 * (std::clock() - startTime) / CLOCKS_PER_SEC;

    // Trys and Steps
    int trys = manager.getIntegrator().getNumStepsAttempted();
    int steps = manager.getIntegrator().getNumStepsTaken();

    // Output
    cout << "trys = " << trys << "\t\tsteps = " << steps;
    cout << "\t\tcpu time = " << runTime << endl;

    // Visuals Replay
    //if (showVisuals) {
    //    //visuals->pollForReplay();
    //    auto vis = model->getVisualizer();
    //    vis.show(state);
    //}

    // Write the model to file
    model->print("C:\\Users\\fcand\\Documents\\block.osim");
}

//_____________________________________________________________________________
/* Entry Point (i.e., main())

The motion of a 10 kg, 6 degree-of-freedom block and its force interaction
with a laboratory floor are simulated.

Contact with the floor is modeled using either
    1) 8 ExponentialSpringForce instances, one at each corner of the block, or
    2) 8 HuntCrossleyForce instances, one at each corner of the block.

For a side-by-side comparison of simulated motions, two blocks (one using
the ExponentialSpringForce class for contact and the other using the
HuntCrossleyForce class) can be created and visualized simultaneously.

For an assessment of computational performance, just one block should be
simulated at a time. Number of integration trys and steps, as well as cpu time,
are reported.

Choice of initial conditions can be made in order to generate the following
motions:
    1) Static (y = 0.1 m, sitting at rest on the floor)
    2) Bouncing (y = 1.0 m, dropped)
    3) Sliding (y = 0.2 m, vx = -2.0 m/s)
    4) Spinning & Sliding (y = 0.2 m, vx = -2.0 m/s, wy = 2.0 pi rad/sec)
    5) Tumbling (py = 2.0 m, vx = -2.0 m/s, wz = 2.0 pi rad/sec)

Additional options allow the following to be specified:
    NoDamp   Parameters are chosen to eliminate all energy dissipation.
    ApplyFx  A ramping horizontal force (Fx) is applied after 5.0 sec.

If no external force is applied, tf = 5.0 s.

If an external force is applied, tf = 10.0 s and this force ramps up
linearly from a value of fx = 0.0 at t = 5.0 s to a value of
fx = |mass*g| at t = 10.0 s. The force is not ramped up prior to t = 5.0 s
in order to allow the block an opportunity to come more fully to rest.
This ramping profile was done with the "Static" initial condition choice
in mind so that friction models could be evaluated more critically.
In particular, a static block should not start sliding until fx > μₛ Fₙ.

For ExponentialSpringForce, the following things are tested:
    a) instantiation
    b) model initialization
    c) energy conservation
    d) data cache access
    e) realization stage invalidation
    f) reporting
    g) serialization

The HuntCrossleyForce class is tested elsewhere (e.g., see
testContactGeometry.cpp). */
int main(int argc, char** argv) {
    try {
        ExponentialSpringTester tester;
        int status = tester.parseCommandLine(argc, argv);
        if (status < 0) {
            cout << "Exiting..." << endl;
            return 1;
        }
        tester.buildModel();
        tester.simulate();

    } catch (const OpenSim::Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
