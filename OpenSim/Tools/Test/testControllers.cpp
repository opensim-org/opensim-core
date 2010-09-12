// testConstrollers.cpp
// Author:  Ajay Seth
/*
* Copyright (c)  2009, Stanford University. All rights reserved. 
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
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/PointKinematics.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Common/NaturalCubicSpline.h>
#include <OpenSim/Common/FunctionAdapter.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Tools/CorrectionController.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/Control/ControlSetController.h>
#include <OpenSim/Simulation/Control/PrescribedController.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;


//==========================================================================================================
bool testControlSetControllerOnBlock()
{
	bool status = true;

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
	CoordinateSet& jointCoordinateSet = blockToGround.getCoordinateSet();
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
	actuatorControls.append(&control);
	actuatorControls.setMemoryOwner(false);
	actuatorControls.setControlValues(initialTime, controlForce);
	actuatorControls.setControlValues(finalTime, controlForce);
	// Create a control set controller that simply applies controls from a ControlSet
	ControlSetController actuatorController;
	actuatorController.setControlSet(&actuatorControls);

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
	integrator.setMaximumStepSize(100);
	integrator.setMinimumStepSize(1.0e-6);
	integrator.setAccuracy(accuracy);
	integrator.setAbsoluteTolerance(1.0e-4);
	Manager manager(osimModel, integrator);

	// Integrate from initial time to final time
	manager.setInitialTime(initialTime);
	manager.setFinalTime(finalTime);
	std::cout<<"\n\nIntegrating from "<<initialTime<<" to "<<finalTime<<std::endl;
	manager.integrate(si);

	si.getQ().dump("Final position:");
	double x_err = fabs(coordinates[0].getValue(si) - 0.5*(controlForce[0]/blockMass)*finalTime*finalTime);
	if (x_err > accuracy){
		cout << "ControlSetControllerOnBlock failed to produce the expected motion." << endl;
		status = false;
	}

	// Save the simulation results
	Storage states(manager.getStateStorage());
	states.print("block_push.sto");

	osimModel.disownAllComponents();

	return status;
}// end of testControlSetControllerOnBlock()


//==========================================================================================================
bool testPrescribedControllerOnBlock()
{
	bool status = true;

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
	CoordinateSet& jointCoordinateSet = blockToGround.getCoordinateSet();
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
	actuatorController.setActuators(osimModel.updActuators());
	actuatorController.prescribeControlForActuator(0, new Constant(controlForce));

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
	integrator.setMaximumStepSize(100);
	integrator.setMinimumStepSize(1.0e-6);
	integrator.setAccuracy(accuracy);
	integrator.setAbsoluteTolerance(1.0e-4);
	Manager manager(osimModel, integrator);

	// Integrate from initial time to final time
	manager.setInitialTime(initialTime);
	manager.setFinalTime(finalTime);
	std::cout<<"\n\nIntegrating from "<<initialTime<<" to "<<finalTime<<std::endl;
	manager.integrate(si);

	si.getQ().dump("Final position:");
	double x_err = fabs(coordinates[0].getValue(si) - 0.5*(controlForce/blockMass)*finalTime*finalTime);
	if (x_err > accuracy){
		cout << "PrescribedController failed to produce the expected motion of block." << endl;
		status = false;
	}

	// Save the simulation results
	Storage states(manager.getStateStorage());
	states.print("block_push.sto");

	osimModel.disownAllComponents();

	return status;
}// end of testPrescribedControllerOnBlock()


//==========================================================================================================
bool testCorrectionControllerOnBlock()
{
	bool status = true;

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
	CoordinateSet& jointCoordinateSet = blockToGround.getCoordinateSet();
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
	integrator.setMaximumStepSize(1.0e-3);
	integrator.setMinimumStepSize(1.0e-6);
	integrator.setAccuracy(1.0e-3);
	integrator.setAbsoluteTolerance(1.0e-4);
	Manager manager(osimModel, integrator);

	osimModel.disownAllComponents();

	return status;
}// end of testCorrectionControllerOnBlock()

int main()
{
    int  status = 0;

	if(  !testControlSetControllerOnBlock()) {
        status = 1;
        cout << " testControlSetControllerOnBlock FAILED " << endl;
    }

	if(! testPrescribedControllerOnBlock()) {
        status = 1;
        cout << " testPrescribedControllerOnBlock FAILED " << endl;
    }

	if(  !testCorrectionControllerOnBlock()) {
        status = 1;
        cout << " testCorrectiveControllerOnBlock FAILED " << endl;
    }	


	return status;
}
