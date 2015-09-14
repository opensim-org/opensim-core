/* -------------------------------------------------------------------------- *
 *                     OpenSim:  testContactGeometry.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

//==========================================================================================================
//  testJoints builds OpenSim models using the OpenSim API and builds an equivalent
//  Simbody system using the Simbody API for each test case. A test fails if the
//  OpenSim and Simbody final states of the simulation are not equivelent (norm-err
//  less than 10x integration error tolerance)
//
//  Tests Include:
//      1. Analytical contact sphere-plane geometry 
//      2. Mesh-based sphere on analytical plane geometry
//      3. 
//
//==========================================================================================================
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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/HuntCrossleyForce.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include "SimTKsimbody.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//==========================================================================================================
// Common Parameters for the simulations are just global.
const static double integ_accuracy = 1.0e-5;
const static double duration = 1.0;
const static double interval = 0.01;
const static SimTK::Vec3 gravity_vec = SimTK::Vec3(0, -9.8065, 0);
const static double mass = 1.0;
const static double radius = 0.10;
const static double height = 1.0;
const static string mesh_files[] = {"sphere_10cm_radius.obj", "sphere_10cm_radius.stl", "sphere_10cm_radius.vtp" };
//"10_5_cm_sphere_47700.obj";

//==========================================================================================================

int testBouncingBall(bool useMesh, const std::string mesh_filename="");
int testBallToBallContact(bool useElasticFoundation, bool useMesh1, bool useMesh2);
void compareHertzAndMeshContactResults();

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
    }
    catch (const OpenSim::Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

//==========================================================================================================
// Test Cases
//==========================================================================================================
int testBouncingBall(bool useMesh, const std::string mesh_filename)
{
    // Setup OpenSim model
    Model *osimModel = new Model;

    //OpenSim bodies
    OpenSim::Body& ground = *new OpenSim::Body("ground", SimTK::Infinity,
        Vec3(0), Inertia());
    osimModel->addBody(&ground);

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
    ContactHalfSpace *floor = new ContactHalfSpace(Vec3(0), Vec3(0, 0, -0.5*SimTK_PI), ground, "ground");
    osimModel->addContactGeometry(floor);
    OpenSim::ContactGeometry* geometry;
    if (useMesh){
        geometry = new ContactMesh(mesh_filename, Vec3(0), Vec3(0), ball, "ball");
    }
    else
        geometry = new ContactSphere(radius, Vec3(0), ball, "ball");
    osimModel->addContactGeometry(geometry);

    OpenSim::Force* force;
    if (useMesh)
    {
        // Add an ElasticFoundationForce.
        OpenSim::ElasticFoundationForce::ContactParameters* contactParams = new OpenSim::ElasticFoundationForce::ContactParameters(1.0e6/radius, 1e-5, 0.0, 0.0, 0.0);
        contactParams->addGeometry("ball");
        contactParams->addGeometry("ground");
        force = new OpenSim::ElasticFoundationForce(contactParams);
        osimModel->addForce(force);
    }
    else
    {
        // Add a HuntCrossleyForce.
        OpenSim::HuntCrossleyForce::ContactParameters* contactParams = new OpenSim::HuntCrossleyForce::ContactParameters(1.0e6, 1e-5, 0.0, 0.0, 0.0);
        contactParams->addGeometry("ball");
        contactParams->addGeometry("ground");
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

    //==========================================================================================================
    // Simulate it and see if it bounces correctly.
    cout << "stateY=" << osim_state.getY() << std::endl;

    RungeKuttaMersonIntegrator integrator(osimModel->getMultibodySystem() );
    integrator.setAccuracy(integ_accuracy);
    Manager manager(*osimModel, integrator);

    for (unsigned int i = 0; i < duration/interval; ++i)
    {
        manager.setInitialTime(i*interval);
        manager.setFinalTime((i+1)*interval);
        manager.integrate(osim_state);
        double time = osim_state.getTime();

        osimModel->getMultibodySystem().realize(osim_state, Stage::Acceleration);
        Vec3 pos, vel;

        osimModel->updSimbodyEngine().getPosition(osim_state, osimModel->getBodySet().get("ball"), Vec3(0), pos);
        osimModel->updSimbodyEngine().getVelocity(osim_state, osimModel->getBodySet().get("ball"), Vec3(0), vel);

        double Etot = mass*((-gravity_vec[1])*pos[1] + 0.5*vel[1]*vel[1]);

        //cout << "starting system energy = " << Etot_orig << " versus current energy = " << Etot << endl;
        // contact absorbs and returns energy so make sure not in contact
        if (pos[1] > 2*radius)
        {
            ASSERT_EQUAL(Etot_orig, Etot, 1e-2, __FILE__, __LINE__, "Bouncing ball on plane Failed: energy was not conserved.");
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

    //OpenSim bodies
    OpenSim::Body& ground = *new OpenSim::Body("ground", SimTK::Infinity,
        Vec3(0), Inertia());
    osimModel->addBody(&ground);

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
    if (useElasticFoundation){
        
    }
    else{
        
    }
    if (useElasticFoundation)
    {
        // Add an ElasticFoundationForce.
        OpenSim::ElasticFoundationForce::ContactParameters* contactParams = new OpenSim::ElasticFoundationForce::ContactParameters(1.0e6/(2*radius), 0.001, 0.0, 0.0, 0.0);
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
        OpenSim::HuntCrossleyForce::ContactParameters* contactParams = new OpenSim::HuntCrossleyForce::ContactParameters(1.0e6, 0.001, 0.0, 0.0, 0.0);
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

    //==========================================================================================================
    // Simulate it and see if it bounces correctly.
    cout << "stateY=" << osim_state.getY() << std::endl;

    RungeKuttaMersonIntegrator integrator(osimModel->getMultibodySystem() );
    integrator.setAccuracy(integ_accuracy);
    integrator.setMaximumStepSize(100*integ_accuracy);
    Manager manager(*osimModel, integrator);
    manager.setInitialTime(0.0);
    manager.setFinalTime(duration);
    manager.integrate(osim_state);

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

    // Hertz theory and ElasticFoundation will not be the same, but they should yield similar results, to withing
    Array<double> rms_tols_1(12, nforces);
    CHECK_STORAGE_AGAINST_STANDARD(meshToMesh, hertz, rms_tols_1, __FILE__, __LINE__, "ElasticFoundation FAILED to Match Hertz Contact ");

    // ElasticFoundation on mesh to mesh and mesh to non-mesh should be virtually identical
    Array<double> rms_tols_2(0.5, nforces);
    CHECK_STORAGE_AGAINST_STANDARD(meshToMesh, meshToNoMesh, rms_tols_2, __FILE__, __LINE__, "ElasticFoundation Mesh-Mesh FAILED to match Mesh-noMesh Case ");

    // ElasticFoundation on non-mesh to mesh and mesh to non-mesh should be identical
    Array<double> rms_tols_3(integ_accuracy, nforces);
    CHECK_STORAGE_AGAINST_STANDARD(noMeshToMesh, meshToNoMesh, rms_tols_3, __FILE__, __LINE__, "ElasticFoundation noMesh-Mesh FAILED to match Mesh-noMesh Case ");

}
