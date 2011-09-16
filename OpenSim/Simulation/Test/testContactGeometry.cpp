// testContactGeometry.cpp
// Author:  Peter Eastman, Ajay Seth
/*
* Copyright (c) 2005-2010, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//==========================================================================================================
//	testJoints builds OpenSim models using the OpenSim API and builds an equivalent
//  Simbody system using the Simbody API for each test case. A test fails if the
//  OpenSim and Simbody final states of the simulation are not equivelent (norm-err
//  less than 10x integration error tolerance)
//
//	Tests Include:
//      1. Analytical contact sphere-plane geometry 
//      2. Mesh-based sphere on analytical plane geometry
//		3. 
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
const static string mesh_file = "sphere_10cm_radius.obj"; //"10_5_cm_sphere_47700.obj";

//==========================================================================================================

int testBouncingBall(bool useMesh);
int testBallToBallContact(bool useElasticFoundation, bool useMesh1, bool useMesh2);

int main()
{
    try
    {
    	testBouncingBall(false);
    	testBouncingBall(true);
		testBallToBallContact(false, false, false);
		testBallToBallContact(true, true, false);
		testBallToBallContact(true, false, true);
		testBallToBallContact(true, true, true);
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
int testBouncingBall(bool useMesh)
{
	// Setup OpenSim model
	Model *osimModel = new Model;

	//OpenSim bodies
    OpenSim::Body& ground = osimModel->getGroundBody();
	OpenSim::Body ball;
	ball.setName("ball");
	ball.setMass(mass);
	ball.setMassCenter(Vec3(0));
	ball.setInertia(Inertia(1.0));

	// Add joints
	FreeJoint free("free", ground, Vec3(0), Vec3(0), ball, Vec3(0), Vec3(0));
	osimModel->addBody(&ball);
	// BAD: have to set memoryOwner to false or model will try to delete
	osimModel->updBodySet().setMemoryOwner(false);

    // Create ContactGeometry.
    ContactHalfSpace *floor = new ContactHalfSpace(Vec3(0), Vec3(0, 0, -0.5*SimTK_PI), ground, "ground");
    osimModel->addContactGeometry(floor);
    OpenSim::ContactGeometry* geometry;
    if (useMesh)
        geometry = new ContactMesh(mesh_file, Vec3(0), Vec3(0), ball, "ball");
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
	osimModel->copy()->print("TestContactGeomtery_Ball.osim");

	Kinematics* kin = new Kinematics(osimModel);
	osimModel->addAnalysis(kin);

    SimTK::State osim_state = osimModel->initSystem();
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
    OpenSim::Body& ground = osimModel->getGroundBody();
	OpenSim::Body ball;
	ball.setName("ball");
	ball.setMass(mass);
	ball.setMassCenter(Vec3(0));
	ball.setInertia(Inertia(1.0));

	// Add joints
	FreeJoint free("free", ground, Vec3(0), Vec3(0), ball, Vec3(0), Vec3(0));

	osimModel->addBody(&ball);
	// BAD: have to set memoryOwner to false or model will try to delete
	osimModel->updBodySet().setMemoryOwner(false);

    // Create ContactGeometry.
    OpenSim::ContactGeometry *ball1, *ball2;

	if (useElasticFoundation && useMesh1)
        ball1 = new ContactMesh(mesh_file, Vec3(0), Vec3(0), ground, "ball1");
    else
        ball1 = new ContactSphere(radius, Vec3(0), ground, "ball1");

    if (useElasticFoundation && useMesh2)
        ball2 = new ContactMesh(mesh_file, Vec3(0), Vec3(0), ball, "ball2");
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

	force->setName(prefix);
	osimModel->addForce(force);
	osimModel->setGravity(gravity_vec);

	osimModel->setName(prefix);
	osimModel->copy()->print(prefix+".osim");

	Kinematics* kin = new Kinematics(osimModel);
	osimModel->addAnalysis(kin);

	ForceReporter* reporter = new ForceReporter(osimModel);
	osimModel->addAnalysis(reporter);

    SimTK::State osim_state = osimModel->initSystem();
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

	// model takes ownership of components unless container set is told otherwise
	delete osimModel;

	return 0;
}