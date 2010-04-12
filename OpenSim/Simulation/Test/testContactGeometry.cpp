// testContactGeometry.cpp
// Author:  Peter Eastman
/*
* Copyright (c) 2005-2009, Stanford University. All rights reserved. 
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
//      1. CustomJoint against Simbody built-in Pin and Universal joints
//      2. CustomJoint versus Simbody FunctionBased with spline based functions
//		3. WeldJoint versus Weld Mobilizer by welding bodies to those in test 1.
//		4. Randomized order of bodies in the BodySet (in 3.) to test connectBodies()
//		
//		TODO random branching toplogy.
//     Add tests here as new joint types are added to OpenSim
//
//==========================================================================================================
#include <iostream>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/PointKinematics.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Simulation/Model/ContactGeometrySet.h>
#include <OpenSim/Simulation/Model/ContactHalfSpace.h>
#include <OpenSim/Simulation/Model/ContactMesh.h>
#include <OpenSim/Simulation/Model/ContactSphere.h>
#include <OpenSim/Simulation/Model/ElasticFoundationForce.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/HuntCrossleyForce.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/TransformAxis.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include "SimTKsimbody.h"
#include "SimTKsimbody_aux.h"
#include "SimTKmath.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

#define ASSERT(cond) {if (!(cond)) throw(exception());}
#define ASSERT_EQUAL(expected, found, tolerance) {double tol = std::max((tolerance), std::abs((expected)*(tolerance))); if ((found)<(expected)-(tol) || (found)>(expected)+(tol)) throw(exception());}

//==========================================================================================================
// Common Parameters for the simulations are just global.
const static double integ_accuracy = 1.0e-4;
const static double duration = 1.0;
const static Vec3 gravity_vec = Vec3(0, -9.8065, 0);
const static double radius = 0.5;
//==========================================================================================================

//==========================================================================================================
// Test Cases
//==========================================================================================================
int testBouncingBall(bool useMesh)
{
	//==========================================================================================================
	// Setup OpenSim model
	Model *osimModel = new Model;
	SimbodyEngine engine;
	//OpenSim bodies
    OpenSim::Body& ground = osimModel->getGroundBody();
	OpenSim::Body ball;
	ball.setName("ball");

	// Add joints
	FreeJoint free("", ground, Vec3(0), Vec3(0), ball, Vec3(0), Vec3(0), true);

	// Rename coordinates for a free joint
	CoordinateSet &free_coords = free.getCoordinateSet();
	for(int i=0; i<free_coords.getSize(); i++){
		std::stringstream coord_name;
		coord_name << "free_q" << i;
		free_coords.get(i).setName(coord_name.str());
		free_coords.get(i).setMotionType(i > 2 ? Coordinate::Translational : Coordinate::Rotational);
	}

	osimModel->addBody(&ball);

    // Create ContactGeometry.
    ContactHalfSpace floor(Vec3(0), Vec3(0, 0, -0.5*SimTK_PI), ground, "ground");
    osimModel->updContactGeometrySet().append(&floor);
    OpenSim::ContactGeometry* geometry;
    if (useMesh)
        geometry = new ContactMesh("cube.obj", Vec3(0), Vec3(0), ball, "ball");
    else
        geometry = new ContactSphere(radius, Vec3(0), ball, "ball");
    osimModel->updContactGeometrySet().append(geometry);

    OpenSim::Force* force;
    if (useMesh)
    {
	    // Add an ElasticFoundationForce.
        OpenSim::ElasticFoundationForce::ContactParameters* contactParams = new OpenSim::ElasticFoundationForce::ContactParameters(1.0e6, 0.001, 0.0, 0.0, 0.0);
        contactParams->addGeometry("ball");
        contactParams->addGeometry("ground");
        force = new OpenSim::ElasticFoundationForce(contactParams);
	    osimModel->updForceSet().append(force);
    }
    else
    {
	    // Add a HuntCrossleyForce.
        OpenSim::HuntCrossleyForce::ContactParameters* contactParams = new OpenSim::HuntCrossleyForce::ContactParameters(1.0e6, 0.001, 0.0, 0.0, 0.0);
        contactParams->addGeometry("ball");
        contactParams->addGeometry("ground");
        force = new OpenSim::HuntCrossleyForce(contactParams);
	    osimModel->updForceSet().append(force);
    }

	// BAD: have to set memoryOwner to false or program will crash when this test is complete.
	osimModel->updBodySet().setMemoryOwner(false);

	ball.setMass(1.0);
	ball.setMassCenter(Vec3(0));
	ball.setInertia(Inertia(1.0));

	osimModel->setGravity(gravity_vec);

	//osimModel = (Model *)osimModel->copy()->print("BouncingBallModel.osim");
	//osimModel->setup();
	//delete osimModel;
	//osimModel = new Model("BouncingBallModel.osim");

    SimTK::State osim_state = osimModel->initSystem();
    osimModel->getSystem().realize(osim_state, Stage::Position );
    osim_state.updQ()[4] = 5.0;

	//==========================================================================================================
	// Simulate it and see if it bounces correctly.

    RungeKuttaMersonIntegrator integrator(osimModel->getSystem() );
    Manager manager(*osimModel, integrator);
    manager.setInitialTime(0.0);
    manager.setFinalTime(0.1);
    for (unsigned int i = 0; i < 100; ++i)
    {
        double time = 0.1*(i+1);
        manager.integrate(osim_state);
        osimModel->getSystem().realize(osim_state, Stage::Acceleration);
        Vec3 pos;
        osimModel->updSimbodyEngine().getPosition(osim_state, osimModel->getBodySet().get("ball"), Vec3(0), pos);
        double y = 5.0+0.5*gravity_vec[1]*time*time;
        if (y > radius)
        {
            ASSERT_EQUAL(y, pos[1], 1e-5);
        }
        else
        {
            ASSERT(pos[1] < 5.0 && pos[1] > 0);
        }
        ASSERT_EQUAL(0.0, pos[0], 1e-3);
        ASSERT_EQUAL(0.0, pos[2], 1e-3);
    }
    delete force;
    delete geometry;
	return 0;
}

int main()
{
    try
    {
    	testBouncingBall(false);
    	testBouncingBall(true);
    }
    catch (std::exception ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }

	return 0;
}
