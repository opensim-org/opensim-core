/* -------------------------------------------------------------------------- *
 *                         OpenSim:  mainFatigue.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Jeffrey A. Reinbolt, Ayman Habib, Ajay Seth, Jack Middleton,    * 
 *            Samuel R. Hamner, Peter Loan                                    *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/* 
 *  Below is an example of an OpenSim application that provides its own 
 *  main() routine.  This application is a forward simulation of a tug-of-war between two
 *  muscles pulling on a block. One of the muscles fatigues and the other does not.
 */

// Author:  Jeff Reinbolt, Ayman Habib, Ajay Seth, Jack Middleton, Samuel Hamner, Peter Loan

//==============================================================================
//==============================================================================
#include <OpenSim/OpenSim.h>
#include <ctime>    // for clock()

#include "LiuThelen2003Muscle.h"

using namespace OpenSim;
using namespace SimTK;

//______________________________________________________________________________
/**
 * Run a simulation of block sliding with contact on by two muscles sliding with contact 
 */
int main()
{
    std::clock_t startTime = std::clock();

	try {
		///////////////////////////////////////////////
		// DEFINE THE SIMULATION START AND END TIMES //
		///////////////////////////////////////////////
		// Define the initial and final simulation times
		double initialTime = 0.0;
		double finalTime = 1.0;

		///////////////////////////////////////////
		// DEFINE BODIES AND JOINTS OF THE MODEL //
		///////////////////////////////////////////
		// Create an OpenSim model and set its name
		Model osimModel;
		osimModel.setName("tugOfWar");

		// GROUND BODY

		// Get a reference to the model's ground body
		OpenSim::Body& ground = osimModel.getGroundBody();

		// Add display geometry to the ground to visualize in the GUI
		ground.addDisplayGeometry("ground.vtp");
		ground.addDisplayGeometry("anchor1.vtp");
		ground.addDisplayGeometry("anchor2.vtp");

		// BLOCK BODY

		// Specify properties of a 20 kg, 0.1 m^3 block body
		double blockMass = 20.0, blockSideLength = 0.1;
		Vec3 blockMassCenter(0);
		Inertia blockInertia = blockMass*Inertia::brick(blockSideLength, blockSideLength, blockSideLength);

		// Create a new block body with the specified properties
		OpenSim::Body *block = new OpenSim::Body("block", blockMass, blockMassCenter, blockInertia);

		// Add display geometry to the block to visualize in the GUI
		block->addDisplayGeometry("block.vtp");

		// FREE JOINT

		// Create a new free joint with 6 degrees-of-freedom (coordinates) between the block and ground bodies
		double halfLength = blockSideLength/2.0;
		Vec3 locationInParent(0, halfLength, 0), orientationInParent(0);
		Vec3 locationInBody(0, halfLength, 0), orientationInBody(0);
		FreeJoint *blockToGround = new FreeJoint("blockToGround", ground, locationInParent, orientationInParent, *block, locationInBody, orientationInBody);
		
		// Get a reference to the coordinate set (6 degrees-of-freedom) between the block and ground bodies
		CoordinateSet& jointCoordinateSet = blockToGround->upd_CoordinateSet();

		// Set the angle and position ranges for the coordinate set
		double angleRange[2] = {-SimTK::Pi/2, SimTK::Pi/2};
		double positionRange[2] = {-1, 1};
		jointCoordinateSet[0].setRange(angleRange);
		jointCoordinateSet[1].setRange(angleRange);
		jointCoordinateSet[2].setRange(angleRange);
		jointCoordinateSet[3].setRange(positionRange);
		jointCoordinateSet[4].setRange(positionRange);
		jointCoordinateSet[5].setRange(positionRange);

		// Add the block body to the model
		osimModel.addBody(block);


		///////////////////////////////////////
		// DEFINE FORCES ACTING ON THE MODEL //
		///////////////////////////////////////
		// MUSCLE FORCES

		// Create two new muscles
		double maxIsometricForce= 1000.0, optimalFiberLength = 0.2, tendonSlackLength = 0.2;
		double pennationAngle = 0.0, activationConstant = 0.015, deactivationConstant = 0.05, fatigueFactor = 0.30, recoveryFactor = 0.20;

		// muscle 1 (model with fatigue)
		LiuThelen2003Muscle* muscle1 = new LiuThelen2003Muscle("Liu",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle, fatigueFactor, recoveryFactor);
		muscle1->setActivationTimeConstant(activationConstant);
		muscle1->setDeactivationTimeConstant(deactivationConstant);
		// muscle 2 (model without fatigue)
		Thelen2003Muscle* muscle2 = new Thelen2003Muscle("Thelen",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);
		muscle2->setActivationTimeConstant(activationConstant);
		muscle2->setDeactivationTimeConstant(deactivationConstant);

		// Define the path of the muscles
		muscle1->addNewPathPoint("Liu-point1", ground, SimTK::Vec3(0.0, halfLength, -0.35));
		muscle1->addNewPathPoint("Liu-point2", *block, SimTK::Vec3(0.0, halfLength, -halfLength));

		muscle2->addNewPathPoint("Thelen-point1", ground, SimTK::Vec3(0.0, halfLength, 0.35));
		muscle2->addNewPathPoint("Thelen-point2", *block, SimTK::Vec3(0.0, halfLength, halfLength));

		// Define the default states for the two muscles
		// Activation
		muscle1->setDefaultActivation(0.01);
		muscle2->setDefaultActivation(0.01);
		// Fiber length
		muscle2->setDefaultFiberLength(0.1);
		muscle1->setDefaultFiberLength(0.1);

		// Add the two muscles (as forces) to the model
		osimModel.addForce(muscle1);
		osimModel.addForce(muscle2);

		///////////////////////////////////
		// DEFINE CONTROLS FOR THE MODEL //
		///////////////////////////////////
		// Create a prescribed controller that simply supplies controls as a function of time.
		// For muscles, controls are normalized motor-neuron excitations
		PrescribedController *muscleController = new PrescribedController();
		muscleController->setActuators(osimModel.updActuators());
		// Define linear "ramp" for the control values for the two muscles
		Array<double> slopeAndIntercept(0.0, 2);  // array of 2 doubles, with 0.0 as default
	
		// muscle control has slope of 1.0 if simulation period is 1s and intercept of 0.
		slopeAndIntercept[0] = 1.0/(finalTime-initialTime);  

		// Set the prescribed muscle controller to use the same muscle control function for each muscle
		muscleController->prescribeControlForActuator("Liu", new LinearFunction(slopeAndIntercept));
		muscleController->prescribeControlForActuator("Thelen", new LinearFunction(slopeAndIntercept));

		// Add the muscle controller to the model
		osimModel.addController(muscleController);

		// Obtain the default acceleration due to gravity
		Vec3 gravity = osimModel.getGravity();

		// Turn on the visualizer to view the simulation run live.
		osimModel.setUseVisualizer(false);

		//////////////////////////
		// PERFORM A SIMULATION //
		//////////////////////////

		// Initialize the system and get the state
		SimTK::State& si = osimModel.initSystem();

		// Init coords to 0 and lock the rotational degrees of freedom so the block doesn't twist
		CoordinateSet& coordinates = osimModel.updCoordinateSet();
		coordinates[0].setValue(si, 0);
		coordinates[1].setValue(si, 0);
		coordinates[2].setValue(si, 0);
		coordinates[3].setValue(si, 0);
		coordinates[4].setValue(si, 0); 
		coordinates[5].setValue(si, 0);
		coordinates[0].setLocked(si, true);
		coordinates[1].setLocked(si, true);
		coordinates[2].setLocked(si, true);
		coordinates[4].setLocked(si, true); // don't let the block fall through or rise off the ground

		// Compute initial conditions for muscles
		osimModel.equilibrateMuscles(si);

		// Create the integrator, force reporter, and manager for the simulation.
		// Create the integrator
		SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
		integrator.setAccuracy(1.0e-4);
		
		// Create the force reporter
		ForceReporter* reporter = new ForceReporter(&osimModel);
		osimModel.updAnalysisSet().adoptAndAppend(reporter);
		// Create the manager
		Manager manager(osimModel, integrator);

		// Print out details of the model
		osimModel.printDetailedInfo(si, std::cout);

		// Integrate from initial time to final time
		manager.setInitialTime(initialTime);
		manager.setFinalTime(finalTime);
		std::cout<<"\nIntegrating from "<<initialTime<<" to "<<finalTime<<std::endl;
		manager.integrate(si);

		//////////////////////////////
		// SAVE THE RESULTS TO FILE //
		//////////////////////////////

		// Save the simulation results
		// Save the states
		manager.getStateStorage().print("tugOfWar_fatigue_states.sto");

		// Save the forces
		reporter->getForceStorage().print("tugOfWar_forces.mot");

		// Save the OpenSim model to a file
		osimModel.print("tugOfWar_fatigue_model.osim");

	}
    catch (const std::exception& ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
        return 1;
    }

    std::cout << "main() routine time = " << 1.e3*(std::clock()-startTime)/CLOCKS_PER_SEC << "ms\n";

    std::cout << "OpenSim example completed successfully.\n";
	return 0;
}
