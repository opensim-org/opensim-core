/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testControllers.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

//==========================================================================================================
//	testControllers builds OpenSim models using the OpenSim API and verifies that controllers
//  behave as described.
//
//	Tests Include:
//		1. Test a control set controller on a block with an ideal actuator
//		2. Test a corrective controller on a block with an ideal actuator
//      
//     Add tests here as new controller types are added to OpenSim
//
//==========================================================================================================
#include <iostream>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/ControllerSet.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/PointKinematics.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Common/FunctionAdapter.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Tools/CorrectionController.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/Control/ControlSetController.h>
#include <OpenSim/Simulation/Control/PrescribedController.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void testControlSetControllerOnBlock();
void testPrescribedControllerOnBlock(bool disabled);
void testCorrectionControllerOnBlock();

int main()
{
    try {
		cout << "Testing ControlSetController" << endl; 
		testControlSetControllerOnBlock();
		cout << "Testing PrescribedController" << endl; 
		testPrescribedControllerOnBlock(false);
		testPrescribedControllerOnBlock(true);
		cout << "Testing CorrectionController" << endl; 
		testCorrectionControllerOnBlock();
    }	
	catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

//==========================================================================================================
void testControlSetControllerOnBlock()
{
	using namespace SimTK;

	// Create a new OpenSim model
	Model osimModel;
	osimModel.setName("osimModel");

	// Get the ground body
	OpenSim::Body& ground = osimModel.getGroundBody();

	// Create a 20 kg, 0.1 m^3 block body
	double blockMass = 20.0, blockSideLength = 0.1;
	Vec3 blockMassCenter(0), groundOrigin(0), blockInGround(0, blockSideLength/2, 0);
	Inertia blockIntertia = Inertia::brick(blockSideLength, blockSideLength, blockSideLength);

	OpenSim::Body block("block", blockMass, blockMassCenter, blockMass*blockIntertia);

	//Create a free joint with 6 degrees-of-freedom
	SimTK::Vec3 noRotation(0);
	SliderJoint blockToGround("",ground, blockInGround, noRotation, block, blockMassCenter, noRotation);
	// Create 6 coordinates (degrees-of-freedom) between the ground and block
	CoordinateSet& jointCoordinateSet = blockToGround.upd_CoordinateSet();
	double posRange[2] = {-1, 1};
	jointCoordinateSet[0].setName("xTranslation");
	jointCoordinateSet[0].setMotionType(Coordinate::Translational);
	jointCoordinateSet[0].setRange(posRange);

	// Add the block body to the model
	osimModel.addBody(&block);

	// Define a single coordinate actuator.
	CoordinateActuator actuator(jointCoordinateSet[0].getName());
	actuator.setName("actuator");

	// Add the actuator to the model
	osimModel.addForce(&actuator);

	double initialTime = 0;
	double finalTime = 1.0;

	// Define the initial and final control values
	double controlForce[1] = {100};
	// Create two control signals
	ControlLinear control;
	control.setName("actuator");
	// Create a control set and add the controls to the set
	ControlSet actuatorControls;
	actuatorControls.adoptAndAppend(&control);
	actuatorControls.setMemoryOwner(false);
	actuatorControls.setControlValues(initialTime, controlForce);
	actuatorControls.setControlValues(finalTime, controlForce);
	// Create a control set controller that simply applies controls from a ControlSet
	ControlSetController actuatorController;
	// Make a copy and set it on the ControlSetController as it takes ownership of the 
	// ControlSet passed in
	actuatorController.setControlSet((ControlSet*)Object::SafeCopy(&actuatorControls));

	// add the controller to the model
	osimModel.addController(&actuatorController);

	// Initialize the system and get the state representing the state system
	SimTK::State& si = osimModel.initSystem();

	// Specify zero slider joint kinematic states
	CoordinateSet &coordinates = osimModel.updCoordinateSet();
	coordinates[0].setValue(si, 0.0);    // x translation
	coordinates[0].setSpeedValue(si, 0.0);			 // x speed

	// Create the integrator and manager for the simulation.
	double accuracy = 1.0e-3;
	SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
	integrator.setAccuracy(accuracy);
	Manager manager(osimModel, integrator);

	// Integrate from initial time to final time
	manager.setInitialTime(initialTime);
	manager.setFinalTime(finalTime);
	std::cout<<"\n\nIntegrating from "<<initialTime<<" to "<<finalTime<<std::endl;
	manager.integrate(si);

	si.getQ().dump("Final position:");
	double x_err = fabs(coordinates[0].getValue(si) - 0.5*(controlForce[0]/blockMass)*finalTime*finalTime);
	ASSERT(x_err <= accuracy, __FILE__, __LINE__, "ControlSetControllerOnBlock failed to produce the expected motion.");

	// Save the simulation results
	Storage states(manager.getStateStorage());
	states.print("block_push.sto");

	osimModel.disownAllComponents();
}// end of testControlSetControllerOnBlock()


//==========================================================================================================
void testPrescribedControllerOnBlock(bool disabled)
{
	using namespace SimTK;

	// Create a new OpenSim model
	Model osimModel;
	osimModel.setName("osimModel");

	// Get the ground body
	OpenSim::Body& ground = osimModel.getGroundBody();

	// Create a 20 kg, 0.1 m^3 block body
	double blockMass = 20.0, blockSideLength = 0.1;
	Vec3 blockMassCenter(0), groundOrigin(0), blockInGround(0, blockSideLength/2, 0);
	Inertia blockIntertia = Inertia::brick(blockSideLength, blockSideLength, blockSideLength);

	OpenSim::Body block("block", blockMass, blockMassCenter, blockMass*blockIntertia);

	//Create a free joint with 6 degrees-of-freedom
	SimTK::Vec3 noRotation(0);
	SliderJoint blockToGround("",ground, blockInGround, noRotation, block, blockMassCenter, noRotation);
	// Create 6 coordinates (degrees-of-freedom) between the ground and block
	CoordinateSet& jointCoordinateSet = blockToGround.upd_CoordinateSet();
	double posRange[2] = {-1, 1};
	jointCoordinateSet[0].setName("xTranslation");
	jointCoordinateSet[0].setMotionType(Coordinate::Translational);
	jointCoordinateSet[0].setRange(posRange);

	// Add the block body to the model
	osimModel.addBody(&block);

	// Define a single coordinate actuator.
	CoordinateActuator actuator(jointCoordinateSet[0].getName());
	actuator.setName("actuator");

	// Add the actuator to the model
	osimModel.addForce(&actuator);

	double initialTime = 0;
	double finalTime = 1.0;

	// Define the initial and final control values
	double controlForce = 100;

	// Create a prescribed controller that simply applies a function of the force
	PrescribedController actuatorController;
	actuatorController.setName("testPrescribedController");
	actuatorController.setActuators(osimModel.updActuators());
	actuatorController.prescribeControlForActuator(0, new Constant(controlForce));
	actuatorController.setDisabled(disabled);

	// add the controller to the model
	osimModel.addController(&actuatorController);

	osimModel.print("blockWithPrescribedController.osim");
	Model modelfileFromFile("blockWithPrescribedController.osim");

	// Verify that serialization and then deserialization of the disable flag is correct
	ASSERT(modelfileFromFile.getControllerSet().get("testPrescribedController").isDisabled() == disabled);

	// Initialize the system and get the state representing the state system
	SimTK::State& si = osimModel.initSystem();

	// Specify zero slider joint kinematic states
	CoordinateSet &coordinates = osimModel.updCoordinateSet();
	coordinates[0].setValue(si, 0.0);    // x translation
	coordinates[0].setSpeedValue(si, 0.0);			 // x speed

	// Create the integrator and manager for the simulation.
	double accuracy = 1.0e-3;
	SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
	integrator.setAccuracy(accuracy);
	Manager manager(osimModel, integrator);

	// Integrate from initial time to final time
	manager.setInitialTime(initialTime);
	manager.setFinalTime(finalTime);
	std::cout<<"\n\nIntegrating from "<<initialTime<<" to "<<finalTime<<std::endl;
	manager.integrate(si);

	si.getQ().dump("Final position:");

	double expected = disabled ? 0 : 0.5*(controlForce/blockMass)*finalTime*finalTime;
	ASSERT_EQUAL(expected, coordinates[0].getValue(si), accuracy, __FILE__, __LINE__, "PrescribedController failed to produce the expected motion of block.");

	// Save the simulation results
	Storage states(manager.getStateStorage());
	states.print("block_push.sto");

	osimModel.disownAllComponents();
}// end of testPrescribedControllerOnBlock()


//==========================================================================================================
void testCorrectionControllerOnBlock()
{
	using namespace SimTK;

	// Create a new OpenSim model
	Model osimModel;
	osimModel.setName("osimModel");

	// Get the ground body
	OpenSim::Body& ground = osimModel.getGroundBody();

	// Create a 20 kg, 0.1 m^3 block body
	double blockMass = 20.0, blockSideLength = 0.1;
	Vec3 blockMassCenter(0), groundOrigin(0), blockInGround(0, blockSideLength/2, 0);
	Inertia blockIntertia = Inertia::brick(blockSideLength, blockSideLength, blockSideLength);

	OpenSim::Body block("block", blockMass, blockMassCenter, blockMass*blockIntertia);

	//Create a free joint with 6 degrees-of-freedom
	SimTK::Vec3 noRotation(0);
	SliderJoint blockToGround("",ground, blockInGround, noRotation, block, blockMassCenter, noRotation);
	// Create 6 coordinates (degrees-of-freedom) between the ground and block
	CoordinateSet& jointCoordinateSet = blockToGround.upd_CoordinateSet();
	double posRange[2] = {-1, 1};
	jointCoordinateSet[0].setName("xTranslation");
	jointCoordinateSet[0].setMotionType(Coordinate::Translational);
	jointCoordinateSet[0].setRange(posRange);

	// Add the block body to the model
	osimModel.addBody(&block);

	// Generate tracking data
	Storage *desiredXTranslation = new Storage();

	CorrectionController tracker;

	// add the controller to the model
	osimModel.addController(&tracker);

	// Initialize the system and get the state representing the state system
	SimTK::State& si = osimModel.initSystem();

	// Create the integrator and manager for the simulation.
	SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
	integrator.setAccuracy(1.0e-4);
	Manager manager(osimModel, integrator);

	osimModel.disownAllComponents();
}// end of testCorrectionControllerOnBlock()