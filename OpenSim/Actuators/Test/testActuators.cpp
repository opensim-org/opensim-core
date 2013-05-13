/* -------------------------------------------------------------------------- *
 *                        OpenSim:  testActuators.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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
//========================  Actuators Tested ==================================
//
//	Tests Include:
//      1. ClutchedPathSpring
//		
//     Add tests here as Actuators are added to OpenSim
//
//=============================================================================
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/OpenSim.h>
#include <OpenSim/Actuators/ClutchedPathSpring.h>
#include <OpenSim/Common/PiecewiseConstantFunction.h>

using namespace OpenSim;
using namespace std;

//==============================================================================
// Common Parameters for the simulations are just global.
const static double integ_accuracy = 1.0e-4;
const static double duration = 1.0;
const static SimTK::Vec3 gravity_vec = SimTK::Vec3(0, -9.8065, 0);
//==============================================================================



void testClutchedPathSpring();

int main()
{
	SimTK::Array_<std::string> failures;

	try {testClutchedPathSpring();}
    catch (const std::exception& e){
		cout << e.what() <<endl; failures.push_back("testClutchedPathSpring");
	}

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }
	cout << "Done, testActuators passed." << endl;
}

//==============================================================================
// Test Cases
//==============================================================================

void testClutchedPathSpring()
{
	using namespace SimTK;

	// start timing
	std::clock_t startTime = std::clock();

	double mass = 1;
	double stiffness = 100;
	double dissipation = 0.3;
	double start_h = 0.5;
	double ball_radius = 0.25;

	double omega = sqrt(stiffness/mass);


	// Setup OpenSim model
	Model* model = new Model;
	model->setName("ClutchedPathSpringModel");
	model->setGravity(gravity_vec);

	//OpenSim bodies
    OpenSim::Body* ground = &model->getGroundBody();
	
	// body that acts as the pulley that the path wraps over
	OpenSim::Body* pulleyBody =
		new OpenSim::Body("PulleyBody", mass ,Vec3(0),  mass*Inertia::brick(0.1, 0.1, 0.1));
	
	// body the path spring is connected to at both ends
	OpenSim::Body* block =
		new OpenSim::Body("block", mass ,Vec3(0),  mass*Inertia::brick(0.2, 0.1, 0.1));
	block->addDisplayGeometry("box.vtp");
	block->scale(Vec3(0.2, 0.1, 0.1), false);

	double dh = mass*gravity_vec(1)/stiffness;
	
	WrapCylinder* pulley = new WrapCylinder();
	pulley->setRadius(0.1);
	pulley->setLength(0.05);

	// Add the wrap object to the body, which takes ownership of it
	pulleyBody->addWrapObject(pulley);

	// Add joints
	WeldJoint weld("", *ground, Vec3(0, 1.0, 0), Vec3(0), *pulleyBody, Vec3(0), Vec3(0));
	SliderJoint slider("", *ground, Vec3(0), Vec3(0,0,Pi/2),*block, Vec3(0), Vec3(0,0,Pi/2));

	double positionRange[2] = {-10, 10};
	// Rename coordinates for a slider joint
	CoordinateSet &slider_coords = slider.upd_CoordinateSet();
	slider_coords[0].setName("block_h");
	slider_coords[0].setRange(positionRange);
	slider_coords[0].setMotionType(Coordinate::Translational);

	model->addBody(block);
	model->addBody(pulleyBody);

	ClutchedPathSpring* spring = 
		new ClutchedPathSpring("clutch_spring", stiffness, dissipation, 0.01);

	spring->updGeometryPath().appendNewPathPoint("origin", *block, Vec3(-0.1, 0.0 ,0.0));
	
	int N = 10;
	for(int i=1; i<N; ++i){
		double angle = i*Pi/N;
		double x = 0.1*cos(angle);
		double y = 0.1*sin(angle);
		spring->updGeometryPath().appendNewPathPoint("", *pulleyBody, Vec3(-x, y ,0.0));
	}

	spring->updGeometryPath().appendNewPathPoint("insertion", *block, Vec3(0.1, 0.0 ,0.0));

	// BUG in defining wrapping API requires that the Force containing the GeometryPath be
	// connected to the model before the wrap can be added
	model->addForce(spring);

	PrescribedController* controller = new PrescribedController();
	controller->addActuator(*spring);
	
	// Control greater than 1 or less than 0 should be treated as 1 and 0 respectively.
	double     timePts[4] = {0.0,  5.0, 6.0, 10.0};
	double clutchOnPts[4] = {1.5, -2.0, 0.5,  0.5};

	PiecewiseConstantFunction controlfunc(4, timePts, clutchOnPts);

	controller->prescribeControlForActuator("clutch_spring", &controlfunc);
	model->addController(controller);

	model->print("ClutchedPathSpringModel.osim");

	// Create the force reporter
	ForceReporter* reporter = new ForceReporter(model);
	model->addAnalysis(reporter);

	//model->setUseVisualizer(true);
	SimTK::State& state = model->initSystem();

	slider_coords[0].setValue(state, start_h);
    model->getMultibodySystem().realize(state, Stage::Position );

	//==========================================================================
	// Compute the force and torque at the specified times.

    RungeKuttaMersonIntegrator integrator(model->getMultibodySystem() );
	integrator.setAccuracy(integ_accuracy);
    Manager manager(*model,  integrator);
	manager.setWriteToStorage(true);

    manager.setInitialTime(0.0);

	double final_t = 4.99999;

	manager.setFinalTime(final_t);
	manager.integrate(state);

	// tension is dynamics dependent because controls must be computed
	model->getMultibodySystem().realize(state, Stage::Dynamics);

	// Now check that the force reported by spring
	double model_force = spring->getTension(state);
	double stretch0 = spring->getStretch(state);

	// the tension should be half the weight of the block
	double analytical_force = -0.5*(gravity_vec(1))*mass;

	cout << "Tension is: " << model_force << " and should be: " << analytical_force << endl;

	// error if the block does not reach equilibrium since spring is clamped
	ASSERT_EQUAL(model_force, analytical_force, 10*integ_accuracy);

	// unclamp and continue integrating
	manager.setInitialTime(final_t);
	final_t = 5.99999;
	manager.setFinalTime(final_t);
	manager.integrate(state);

	// tension is dynamics dependent because controls must be computed
	model->getMultibodySystem().realize(state, Stage::Dynamics);

	// tension should go to zero quickly
	model_force = spring->getTension(state);

	cout << "Tension is: " << model_force << " and should be: 0.0" << endl;
	// is unclamped and block should be in free-fall
	ASSERT_EQUAL(model_force, 0.0, 10*integ_accuracy);

	// spring is reclamped at 7s so keep integrating
	manager.setInitialTime(final_t);
	final_t = 10.0;
	manager.setFinalTime(final_t);
	manager.integrate(state);

	// tension is dynamics dependent because controls must be computed
	model->getMultibodySystem().realize(state, Stage::Dynamics);

	// tension should build to support the block again
	model_force = spring->getTension(state);
	double stretch1 = spring->getStretch(state);

	cout << "Tension is: " << model_force << " and should be: "<< analytical_force << endl;

	// is unclamped and block should be in free-fall
	ASSERT_EQUAL(model_force, analytical_force, 10*integ_accuracy);

	cout << "Steady stretch at control = 1.0 is " << stretch0 << " m." << endl;
	cout << "Steady stretch at control = 0.5 is " << stretch1 << " m." << endl;

	ASSERT_EQUAL(2*stretch0, stretch1, 10*integ_accuracy);

	manager.getStateStorage().print("clutched_path_spring_states.sto");
	model->getControllerSet().printControlStorage("clutched_path_spring_controls.sto");

	// Save the forces
	reporter->getForceStorage().print("clutched_path_spring_forces.mot"); 

	// Before exiting lets see if copying the spring works
	ClutchedPathSpring* copyOfSpring = spring->clone();
	ASSERT(*copyOfSpring == *spring);
	
	// Check that de/serialization works
	Model modelFromFile("ClutchedPathSpringModel.osim");
	ASSERT(modelFromFile == *model);

	std::cout << "Test clutched spring time = " << 1.e3*(std::clock()-startTime)/CLOCKS_PER_SEC << "ms\n" << endl;
}