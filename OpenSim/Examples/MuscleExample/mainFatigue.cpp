// mainFatigue.cpp

/* Copyright (c)  2009 Stanford University
 * Use of the OpenSim software in source form is permitted provided that the following
 * conditions are met:
 *   1. The software is used only for non-commercial research and education. It may not
 *     be used in relation to any commercial activity.
 *   2. The software is not distributed or redistributed.  Software distribution is allowed 
 *     only through https://simtk.org/home/opensim.
 *   3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
 *      presentations, or documents describing work in which OpenSim or derivatives are used.
 *   4. Credits to developers may not be removed from executables
 *     created from modifications of the source.
 *   5. Modifications of source code must retain the above copyright notice, this list of
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
		CoordinateSet& jointCoordinateSet = blockToGround->getCoordinateSet();

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
		osimModel.updAnalysisSet().append(reporter);
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
    catch (const OpenSim::Exception &ex)
    {
        std::cout << ex.getMessage() << std::endl;
        return 1;
    }
    catch (const std::exception &ex)
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
