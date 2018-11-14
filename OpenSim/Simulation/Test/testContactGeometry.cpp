/* -------------------------------------------------------------------------- *
 *                     OpenSim:  testContactGeometry.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Eastman, Ajay Seth                                        *
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

//==============================================================================
//
//  Tests Include:
//      1. Analytical contact sphere-plane geometry 
//      2. Mesh-based sphere on analytical plane geometry
//      3. Intermediate frames are handled correctly.
//
//==============================================================================
#include <iostream>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Exception.h>

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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include "SimTKsimbody.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//==============================================================================
// Common Parameters for the simulations are just global.
const static double integ_accuracy = 1.0e-5;
const static double duration = 1.0;
const static double interval = 0.01;
const static SimTK::Vec3 gravity_vec = SimTK::Vec3(0, -9.8065, 0);
const static double mass = 1.0;
const static double radius = 0.10;
const static double height = 1.0;
const static string mesh_files[] = {"sphere_10cm_radius.obj",
                                    "sphere_10cm_radius.stl",
                                    "sphere_10cm_radius.vtp"};
//"10_5_cm_sphere_47700.obj";

//==============================================================================

int testBouncingBall(bool useMesh, const std::string mesh_filename="");
int testBallToBallContact(bool useElasticFoundation, bool useMesh1, bool useMesh2);
void compareHertzAndMeshContactResults();
template <typename ContactType> // e.g., HuntCrossley.
void testIntermediateFrames();

int main()
{
    try
    {
        testBouncingBall(false);
        cout << "Testing mesh .obj format" << endl;
        testBouncingBall(true, mesh_files[0]);
        cout << "Testing mesh .stl format" << endl;
        testBouncingBall(true, mesh_files[1]);
        cout << "Testing mesh .vtp format" << endl;
        testBouncingBall(true, mesh_files[2]);
        testBallToBallContact(false, false, false);
        testBallToBallContact(true, true, false);
        testBallToBallContact(true, false, true);
        testBallToBallContact(true, true, true); 
        compareHertzAndMeshContactResults();

        testIntermediateFrames<OpenSim::HuntCrossleyForce>();
        testIntermediateFrames<OpenSim::ElasticFoundationForce>();
    }
    catch (const OpenSim::Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

//==============================================================================
// Test Cases
//==============================================================================
int testBouncingBall(bool useMesh, const std::string mesh_filename)
{
    // Setup OpenSim model
    Model *osimModel = new Model;

    //OpenSim Ground
    Ground& ground = osimModel->updGround();

    // Add Body
    OpenSim::Body ball;
    ball.setName("ball");
    ball.set_mass(mass);
    ball.set_mass_center(Vec3(0));
    ball.setInertia(Inertia(1.0));

    // Add joints
    FreeJoint free("free", ground, Vec3(0), Vec3(0), ball, Vec3(0), Vec3(0));
    osimModel->addBody(&ball);
    osimModel->addJoint(&free);

    // Create ContactGeometry.
    ContactHalfSpace *floor = new ContactHalfSpace(Vec3(0),
                                                   Vec3(0, 0, -0.5*SimTK_PI),
                                                   ground,
                                                   "floor");
    osimModel->addContactGeometry(floor);
    OpenSim::ContactGeometry* geometry;
    if (useMesh){
        geometry = new ContactMesh(mesh_filename, Vec3(0), Vec3(0),
                                   ball, "sphere");
    }
    else
        geometry = new ContactSphere(radius, Vec3(0), ball, "sphere");
    osimModel->addContactGeometry(geometry);

    OpenSim::Force* force;
    if (useMesh)
    {
        // Add an ElasticFoundationForce.
        auto* contactParams =
            new OpenSim::ElasticFoundationForce::ContactParameters(
                    1.0e6/radius, 1e-5, 0.0, 0.0, 0.0);
        contactParams->addGeometry("sphere");
        contactParams->addGeometry("floor");
        force = new OpenSim::ElasticFoundationForce(contactParams);
        osimModel->addForce(force);
    }
    else
    {
        // Add a HuntCrossleyForce.
        auto* contactParams =
            new OpenSim::HuntCrossleyForce::ContactParameters(
                    1.0e6, 1e-5, 0.0, 0.0, 0.0);
        contactParams->addGeometry("sphere");
        contactParams->addGeometry("floor");
        force = new OpenSim::HuntCrossleyForce(contactParams);
        osimModel->addForce(force);
    }

    osimModel->setGravity(gravity_vec);

    osimModel->setName("TestContactGeomtery_Ball");
    osimModel->clone()->print("TestContactGeomtery_Ball.osim");

    Kinematics* kin = new Kinematics(osimModel);
    osimModel->addAnalysis(kin);

    SimTK::State& osim_state = osimModel->initSystem();
    osim_state.updQ()[4] = height;
    osimModel->getMultibodySystem().realize(osim_state, Stage::Position );

    //Initial system energy is all potential
    double Etot_orig = mass*(-gravity_vec[1])*height;

    //==========================================================================
    // Simulate it and see if it bounces correctly.
    cout << "stateY=" << osim_state.getY() << std::endl;

    Manager manager(*osimModel);
    manager.setIntegratorAccuracy(integ_accuracy);
    osim_state.setTime(0.0);
    manager.initialize(osim_state);

    for (unsigned int i = 0; i < duration/interval; ++i)
    {
        osim_state = manager.integrate((i + 1)*interval);
        double time = osim_state.getTime();

        osimModel->getMultibodySystem().realize(osim_state, Stage::Acceleration);
        Vec3 pos = ball.findStationLocationInGround(osim_state, Vec3(0));
        Vec3 vel = ball.findStationVelocityInGround(osim_state, Vec3(0));

        double Etot = mass*((-gravity_vec[1])*pos[1] + 0.5*vel[1]*vel[1]);

        //cout << "starting system energy = " << Etot_orig << " versus current energy = " << Etot << endl;
        // contact absorbs and returns energy so make sure not in contact
        if (pos[1] > 2*radius)
        {
            ASSERT_EQUAL(Etot_orig, Etot, 1e-2, __FILE__, __LINE__,
                    "Bouncing ball on plane Failed: energy was not conserved.");
        }
        else
        {
            cout << "In contact at time = " << time << endl; 
            ASSERT(pos[1] < 5.0 && pos[1] > 0);
        }
        ASSERT_EQUAL(0.0, pos[0], 1e-4);
        ASSERT_EQUAL(0.0, pos[2], 1e-4);
        ASSERT_EQUAL(0.0, vel[0], 1e-3);
        ASSERT_EQUAL(0.0, vel[2], 1e-3);
    }

    std::string prefix = useMesh?"Kinematics_Mesh":"Kinematics_NoMesh";
    kin->printResults(prefix);

    osimModel->disownAllComponents();
    // model takes ownership of components unless container set is told otherwise
    delete osimModel;

    return 0;
}


// test sphere to sphere contact using elastic foundation with and without 
// meshes and their combination
int testBallToBallContact(bool useElasticFoundation, bool useMesh1, bool useMesh2)
{
    // Setup OpenSim model
    Model *osimModel = new Model;

    //OpenSim Ground
    Ground& ground = osimModel->updGround();

    // Add Body
    OpenSim::Body ball;
    ball.setName("ball");
    ball.setMass(mass);
    ball.setMassCenter(Vec3(0));
    ball.setInertia(Inertia(1.0));

    // Add joints
    FreeJoint free("free", ground, Vec3(0), Vec3(0), ball, Vec3(0), Vec3(0));

    osimModel->addBody(&ball);
    osimModel->addJoint(&free);

    // Create ContactGeometry.
    OpenSim::ContactGeometry *ball1, *ball2;

    if (useElasticFoundation && useMesh1)
        ball1 = new ContactMesh(mesh_files[0], Vec3(0), Vec3(0), ground, "ball1");
    else
        ball1 = new ContactSphere(radius, Vec3(0), ground, "ball1");

    if (useElasticFoundation && useMesh2)
        ball2 = new ContactMesh(mesh_files[0], Vec3(0), Vec3(0), ball, "ball2");
    else
        ball2 = new ContactSphere(radius, Vec3(0), ball, "ball2");
    
    osimModel->addContactGeometry(ball1);
    osimModel->addContactGeometry(ball2);

    OpenSim::Force* force;

    std::string prefix;
    if (useElasticFoundation)
    {
        // Add an ElasticFoundationForce.
        auto* contactParams =
            new OpenSim::ElasticFoundationForce::ContactParameters(
                    1.0e6/(2*radius), 0.001, 0.0, 0.0, 0.0);
        contactParams->addGeometry("ball1");
        contactParams->addGeometry("ball2");
        force = new OpenSim::ElasticFoundationForce(contactParams);
        prefix = "EF_";
        prefix += useMesh1 ?"Mesh":"noMesh";
        prefix += useMesh2 ? "_to_Mesh":"_to_noMesh";
        
    }
    else
    {
        // Add a Hertz HuntCrossleyForce.
        auto* contactParams =
            new OpenSim::HuntCrossleyForce::ContactParameters(
                    1.0e6, 0.001, 0.0, 0.0, 0.0);
        contactParams->addGeometry("ball1");
        contactParams->addGeometry("ball2");
        force = new OpenSim::HuntCrossleyForce(contactParams);
        prefix = "Hertz";
        
    }

    force->setName("contact");
    osimModel->addForce(force);
    osimModel->setGravity(gravity_vec);

    osimModel->setName(prefix);
    osimModel->clone()->print(prefix+".osim");

    Kinematics* kin = new Kinematics(osimModel);
    osimModel->addAnalysis(kin);

    ForceReporter* reporter = new ForceReporter(osimModel);
    osimModel->addAnalysis(reporter);

    SimTK::State& osim_state = osimModel->initSystem();
    osim_state.updQ()[4] = height;
    osimModel->getMultibodySystem().realize(osim_state, Stage::Position );

    //==========================================================================
    // Simulate it and see if it bounces correctly.
    cout << "stateY=" << osim_state.getY() << std::endl;

    Manager manager(*osimModel);
    manager.setIntegratorAccuracy(integ_accuracy);
    manager.setIntegratorMaximumStepSize(100*integ_accuracy);
    osim_state.setTime(0.0);
    manager.initialize(osim_state);
    osim_state = manager.integrate(duration);

    kin->printResults(prefix);
    reporter->printResults(prefix);

    osimModel->disownAllComponents();
    // model takes ownership of components unless container set is told otherwise
    delete osimModel;

    return 0;
}

void compareHertzAndMeshContactResults()
{
    Storage hertz("Hertz_ForceReporter_forces.sto");
    Storage meshToMesh("EF_Mesh_to_Mesh_ForceReporter_forces.sto");
    Storage noMeshToMesh("EF_noMesh_to_Mesh_ForceReporter_forces.sto");
    Storage meshToNoMesh("EF_Mesh_to_noMesh_ForceReporter_forces.sto");

    int nforces = hertz.getColumnLabels().getSize()-1;

    // Hertz theory and ElasticFoundation will not be the same, but they should
    // yield similar results, to withing
    std::vector<double> rms_tols_1(nforces, 12);
    CHECK_STORAGE_AGAINST_STANDARD(meshToMesh, hertz, rms_tols_1,
            __FILE__, __LINE__,
            "ElasticFoundation FAILED to Match Hertz Contact ");

    // ElasticFoundation on mesh to mesh and mesh to non-mesh should be
    // virtually identical
    std::vector<double> rms_tols_2(nforces, 0.5);
    CHECK_STORAGE_AGAINST_STANDARD(meshToMesh, meshToNoMesh, rms_tols_2,
            __FILE__, __LINE__,
            "ElasticFoundation Mesh-Mesh FAILED to match Mesh-noMesh Case ");

    // ElasticFoundation on non-mesh to mesh and mesh to non-mesh should be
    // identical
    std::vector<double> rms_tols_3(nforces, integ_accuracy);
    CHECK_STORAGE_AGAINST_STANDARD(noMeshToMesh, meshToNoMesh, rms_tols_3,
            __FILE__, __LINE__,
            "ElasticFoundation noMesh-Mesh FAILED to match Mesh-noMesh Case ");

}


// In version 4.0, we introduced intermediate PhysicalFrames to
// ContactGeometry. The test below ensures that the intermediate frames (as
// well as the ContactGeometry's location and orientation properties) are
// accounted for by comparing results for equivalent systems, some of which
// use an intermediate offset frame.
// The system is a point mass situated 1 meter along a link that is attached to
// ground by a hinge. The contact ball (sphere or mesh) is 1 meter up and 0.5
// meters to the right of the mass (in the frame of the link). The ball bounces
// on a platform, whose orientation is horizontal (or, 90 degrees clockwise
// from its "default" orientation, which is vertical).
//
//                                       0.5 m
//                                     -----(ball)
//                                     |
//                                     |
//                                     | 1 m
//                             1 m     |
//          ________(hinge)----------(mass)_______________________
//                     ^                      (platform half-space)
//                     |                       must be rotated 90 deg.
//                  origin                     clockwise to achieve the
//                                             horizontal orientation
//                        
// The link starts at an incline of 27 degrees and then the link drops down and
// hits the platform.
// We test three equivalent systems that specify the transforms for the
// platform and ball geometries in different ways:
//    1. Only using WeldJoints and massless bodies.
//    2. Using a mix of PhysicalOffsetFrames and the geometry's location and
//       orientation properties.
//    3. Only using PhysicalOffsetFrames.

// Add contact geometry and contact force components to the model.
template <typename ContactType>
void addContactComponents(Model& model,
        PhysicalFrame& frameForBall, Vec3 locForBall,
        PhysicalFrame& frameForPlatform, Vec3 orientationForPlatform);

// Specialize for Hunt-Crossley.
template<>
void addContactComponents<OpenSim::HuntCrossleyForce>(Model& model,
        PhysicalFrame& frameForBall, Vec3 locForBall,
        PhysicalFrame& frameForPlatform, Vec3 orientationForPlatform) {

    // ContactGeometry.
    // By default, x > 0 is inside the half space.
    auto* floor = new ContactHalfSpace(Vec3(0), orientationForPlatform,
            frameForPlatform, "platform");
    auto* geometry = new ContactSphere(radius, locForBall,
                                       frameForBall, "ball");
    model.addContactGeometry(floor);
    model.addContactGeometry(geometry);

    // Force.
    auto* contactParams = new OpenSim::HuntCrossleyForce::ContactParameters(
            1.0e6, 1e-5, 0.0, 0.0, 0.0);
    contactParams->addGeometry("ball");
    contactParams->addGeometry("platform");
    auto* force = new OpenSim::HuntCrossleyForce(contactParams);
    model.addForce(force);
}

// Specialize for Elastic Foundation.
template<>
void addContactComponents<OpenSim::ElasticFoundationForce>(Model& model,
        PhysicalFrame& frameForBall, Vec3 locForBall,
        PhysicalFrame& frameForPlatform, Vec3 orientationForPlatform) {

    // ContactGeometry.
    // By default, x > 0 is inside the half space.
    auto* floor = new ContactHalfSpace(Vec3(0), orientationForPlatform,
            frameForPlatform, "platform");
    auto* geometry = new ContactMesh(mesh_files[0], locForBall, Vec3(0),
                                     frameForBall, "ball");
    model.addContactGeometry(floor);
    model.addContactGeometry(geometry);

    // Force.
    auto* contactParams =
        new OpenSim::ElasticFoundationForce::ContactParameters(
            1.0e6, 1e-5, 0.0, 0.0, 0.0);
    contactParams->addGeometry("ball");
    contactParams->addGeometry("platform");
    auto* force = new OpenSim::ElasticFoundationForce(contactParams);
    model.addForce(force);
}

Model createBaseModel() {
    Model model;
    // For debugging: model.setUseVisualizer(true);

    // Body and Joint.
    const Ground& ground = model.getGround();
    auto* point = new OpenSim::Body("point", mass, Vec3(0), Inertia(1.0));
    auto* hinge = new OpenSim::PinJoint("hinge",
                               ground, Vec3(0), Vec3(0),
                               *point, Vec3(-1, 0, 0), Vec3(0));

    model.addBody(point);
    model.addJoint(hinge);

    return model;
}

template <typename ContactType>
void testIntermediateFrames() {

    // Simulate the given model for one second and return the final state.
    auto simulate = [](Model& model) -> SimTK::State {
        // Initialize and set state.
        SimTK::State& state = model.initSystem();
        const auto& hinge = model.getJointSet().get("hinge");
        const auto& coord = hinge.getCoordinate();
        coord.setValue(state, 0.15 * SimTK::Pi);

        // Integrate.
        Manager manager(model);
        manager.setIntegratorAccuracy(integ_accuracy);
        state.setTime(0.0);
        manager.initialize(state);
        state = manager.integrate(1.0);

        return state;
    };

    const Real deg90 = 0.5 * SimTK::Pi;
    const Real deg45 = 0.25 * SimTK::Pi;

    // Achieve transforms with weld joints and massless bodies.
    SimTK::State stateWeld;
    {
        Model model = createBaseModel();

        // Scaffolding for the ball.
        const auto& point = model.getBodySet().get("point");
        auto* linkOffset = new OpenSim::Body("link_offset",
                                             0, Vec3(0), Inertia(0.0));
        auto* linkWeld = new OpenSim::WeldJoint("link_weld",
                                   point, Vec3(0), Vec3(0),
                                   // Body is 0.5m to the right and 1m up.
                                   *linkOffset, Vec3(-0.5, -1, 0), Vec3(0));
        model.addBody(linkOffset);
        model.addJoint(linkWeld);

        // Scaffolding for the platform.
        auto* platformOffset = new OpenSim::Body("platform_offset",
                                                 0, Vec3(0), Inertia(0));
        auto* platformWeld = new OpenSim::WeldJoint("platform_weld",
                                   model.getGround(), Vec3(0), Vec3(0),
                                   *platformOffset, Vec3(0), Vec3(0));
        model.addBody(platformOffset);
        model.addJoint(platformWeld);

        addContactComponents<ContactType>(model,
                *linkOffset, Vec3(0),
                *platformOffset, Vec3(0, 0, -deg90));
        stateWeld = simulate(model);

        // Make sure this model actually bounced; otherwise, the test is not
        // meaningful.
        SimTK_TEST(model.calcMassCenterVelocity(stateWeld)[1] > 0);
    }

    // Achieve transforms with a mix of PhysicalOffsetFrames and
    // ContactGeometry's location and orientation properties.
    SimTK::State stateIntermedFrameY;
    {
        Model model = createBaseModel();

        // Scaffolding for the ball.
        auto* linkOffset = new PhysicalOffsetFrame("link_offset",
                model.getBodySet().get("point"),
                // Frame is up 1m in the y direction.
                SimTK::Transform(Vec3(0, 1, 0)));
        model.addComponent(linkOffset);

        // Scaffolding for the platform.
        auto* platformOffset = new PhysicalOffsetFrame("platform_offset",
                model.getGround(),
                SimTK::Transform(SimTK::Rotation(-deg45, SimTK::ZAxis)));
        model.addComponent(platformOffset);

        addContactComponents<ContactType>(model,
                *linkOffset, Vec3(0.5, 0, 0),
                *platformOffset, Vec3(0, 0, -deg45));
        stateIntermedFrameY = simulate(model);

        // Make sure this model actually bounced; otherwise, the test is not
        // meaningful.
        SimTK_TEST(model.calcMassCenterVelocity(stateIntermedFrameY)[1] > 0);
    }

    // Achieve transforms solely with PhysicalOffsetFrames.
    SimTK::State stateIntermedFrameXY;
    {
        Model model = createBaseModel();

        // Scaffolding for the ball.
        auto* linkOffset = new PhysicalOffsetFrame("link_offset",
                model.getBodySet().get("point"),
                // Frame is 0.5m to the right and 1m up.
                SimTK::Transform(Vec3(0.5, 1, 0)));
        model.addComponent(linkOffset);

        // Scaffolding for the platform.
        auto* platformOffset = new PhysicalOffsetFrame("platform_offset",
                model.getGround(),
                SimTK::Transform(SimTK::Rotation(-deg90, SimTK::ZAxis)));
        model.addComponent(platformOffset);

        addContactComponents<ContactType>(model, *linkOffset, Vec3(0),
                                                 *platformOffset, Vec3(0));
        stateIntermedFrameXY = simulate(model);

        // Make sure this model actually bounced; otherwise, the test is not
        // meaningful.
        SimTK_TEST(model.calcMassCenterVelocity(stateIntermedFrameXY)[1] > 0);
    }

    SimTK_TEST_EQ_TOL(stateWeld.getY(), stateIntermedFrameY.getY(), 1e-10);
    SimTK_TEST_EQ_TOL(stateWeld.getY(), stateIntermedFrameXY.getY(), 1e-10);
}


















