// main.cpp

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
 *  main() routine.  This application is a forward simulation of tug-of-war between two
 *  muscles pulling on a block.
 */

// Author:  Jeff Reinbolt, Ayman Habib, Ajay Seth, Jack Middleton, Samuel Hamner

//==============================================================================
//==============================================================================
#include <OpenSim/OpenSim.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

//______________________________________________________________________________
/**
 * Run a simulation of block sliding with contact on by two muscles sliding with contact 
 */
int main(int argc, char **argv)
{
	using namespace SimTK;

	bool waitForUserInput = (argc<2);
	try {
		const clock_t start = clock();

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
		Vec3 locationInParent(0, blockSideLength/2, 0), orientationInParent(0), locationInBody(0), orientationInBody(0);
		FreeJoint *blockToGround = new FreeJoint("blockToGround", ground, locationInParent, orientationInParent, *block, locationInBody, orientationInBody);
		
		// Get a reference to the coordinate set (6 degrees-of-freedom) between the block and ground bodies
		CoordinateSet& jointCoordinateSet = blockToGround->getCoordinateSet();

		// Set the angle and position ranges for the coordinate set
		double angleRange[2] = {-Pi/2, Pi/2};
		double positionRange[2] = {-1, 1};
		jointCoordinateSet[0].setRange(angleRange);
		jointCoordinateSet[1].setRange(angleRange);
		jointCoordinateSet[2].setRange(angleRange);
		jointCoordinateSet[3].setRange(positionRange);
		jointCoordinateSet[4].setRange(positionRange);
		jointCoordinateSet[5].setRange(positionRange);

		// Add the block body to the model
		osimModel.addBody(block);

		///////////////////////////////////////////////
		// DEFINE THE SIMULATION START AND END TIMES //
		///////////////////////////////////////////////

		// Define the initial and final simulation times
		double initialTime = 0.0;
		double finalTime = 4.0;

		/////////////////////////////////////////////
		// DEFINE CONSTRAINTS IMPOSED ON THE MODEL //
		/////////////////////////////////////////////

		// Specify properties of a point on a line constraint to limit the block's motion
		Vec3 lineDirection(1,0,-1);
		Vec3 pointOnLine(1,0,-1);
		Vec3 pointOnFollowerBody(0,-0.05,0);

		// Create a new point on a line constraint
		PointOnLineConstraint *lineConstraint = new PointOnLineConstraint(ground, lineDirection, pointOnLine, *block, pointOnFollowerBody);

		// Add the new point on a line constraint to the model
		osimModel.addConstraint(lineConstraint);

		///////////////////////////////////////
		// DEFINE FORCES ACTING ON THE MODEL //
		///////////////////////////////////////
		// GRAVITY

		// Define the acceleration due to gravity
		osimModel.setGravity(Vec3(0,-9.80665,0));

		// MUSCLE FORCES

		// Create two new muscles
		double maxIsometricForce = 1000.0, optimalFiberLength = 0.1, tendonSlackLength = 0.2, pennationAngle = 0.0, activation = 0.0001, deactivation = 1.0;
		// Create new muscle 1
		Schutte1993Muscle_Deprecated *muscle1 = new Schutte1993Muscle_Deprecated("muscle1",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);
		muscle1->setActivation1(activation);
		muscle1->setActivation2(deactivation);
		// Create new muscle 2
		Thelen2003Muscle *muscle2 = new Thelen2003Muscle("muscle2",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);
		muscle2->setActivationTimeConstant(activation);
		muscle2->setDeactivationTimeConstant(deactivation);

		// Specify the paths for the two muscles
		// Path for muscle 1
		muscle1->addNewPathPoint("muscle1-point1", ground, Vec3(0.0,0.05,-0.35));
		muscle1->addNewPathPoint("muscle1-point2", *block, Vec3(0.0,0.0,-0.05));
		// Path for muscle 2
		muscle2->addNewPathPoint("muscle2-point1", ground, Vec3(0.0,0.05,0.35));
		muscle2->addNewPathPoint("muscle2-point2", *block, Vec3(0.0,0.0,0.05));

		// Add the two muscles (as forces) to the model
		osimModel.addForce(muscle1);
		osimModel.addForce(muscle2);

		// CONTACT FORCE

		// Define contact geometry
		// Create new floor contact halfspace
		ContactHalfSpace *floor = new ContactHalfSpace(Vec3(0), Vec3(0, 0, -0.5*SimTK_PI), ground, "floor");
		// Create new cube contact mesh
		OpenSim::ContactMesh *cube = new OpenSim::ContactMesh("blockRemesh192.obj", Vec3(0), Vec3(0), *block, "cube");

		// Add contact geometry to the model
		osimModel.addContactGeometry(floor);
		osimModel.addContactGeometry(cube);

		// Contact parameters
		double stiffness = 1.0e8, dissipation = 0.01, friction = 0.25;

		// Define contact parameters for elastic foundation force
		OpenSim::ElasticFoundationForce::ContactParameters *contactParams = 
			new OpenSim::ElasticFoundationForce::ContactParameters(stiffness, dissipation, friction, 0, 0);
		contactParams->addGeometry("cube");
		contactParams->addGeometry("floor");
		
		// Create a new elastic foundation (contact) force between the floor and cube.
		OpenSim::ElasticFoundationForce *contactForce = new OpenSim::ElasticFoundationForce(contactParams);
		contactForce->setName("contactForce");

		// Add the new elastic foundation force to the model
		osimModel.addForce(contactForce);

		// PRESCRIBED FORCE

		// Specify properties of a force function to be applied to the block
		double time[2] = {0, finalTime};					// time nodes for linear function
		double fXofT[2] = {0,  -blockMass*9.80665*3.0};		// force values at t1 and t2
		double pXofT[2] = {0, 0.1};							// point in x values at t1 and t2

		// Create a new linear functions for the force and point components
		PiecewiseLinearFunction *forceX = new PiecewiseLinearFunction(2, time, fXofT);
		PiecewiseLinearFunction *pointX = new PiecewiseLinearFunction(2, time, pXofT);

		// Create a new prescribed force applied to the block
		PrescribedForce *prescribedForce = new PrescribedForce(block);
		prescribedForce->setName("prescribedForce");

		// Set the force and point functions for the new prescribed force
		prescribedForce->setForceFunctions(forceX, new Constant(0.0), new Constant(0.0));
		prescribedForce->setPointFunctions(pointX, new Constant(0.0), new Constant(0.0));

		// Add the new prescribed force to the model
		osimModel.addForce(prescribedForce);

		//osimModel.print("tugOfWar_musclesContactPres.osim");

		///////////////////////////////////
		// DEFINE CONTROLS FOR THE MODEL //
		///////////////////////////////////
		// Create a prescribed controller that simply applies controls as function of time
		PrescribedController *muscleController = new PrescribedController();
		muscleController->setActuators(osimModel.updActuators());
		// Define linear functions for the control values for the two muscles
		Array<double> slopeAndIntercept1(0.0, 2);  // array of 2 doubles
		Array<double> slopeAndIntercept2(0.0, 2);
		// muscle1 control has slope of -1 starting 1 at t = 0
		slopeAndIntercept1[0] = -1.0/(finalTime-initialTime);  slopeAndIntercept1[1] = 1.0;
		// muscle2 control has slope of 1 starting 0.05 at t = 0
		slopeAndIntercept2[0] = 1.0/(finalTime-initialTime);  slopeAndIntercept2[1] = 0.05;
		
		// Set the indiviudal muscle control functions for the prescribed muscle controller
		muscleController->prescribeControlForActuator("muscle1", new LinearFunction(slopeAndIntercept1));
		muscleController->prescribeControlForActuator("muscle2", new LinearFunction(slopeAndIntercept2));

		// Add the control set controller to the model
		osimModel.addController(muscleController);

		//////////////////////////
		// PERFORM A SIMULATION //
		//////////////////////////

		// Define the default states for the two muscles
		// Activation
		muscle1->setDefaultActivation(slopeAndIntercept1[1]);
		muscle2->setDefaultActivation(slopeAndIntercept2[1]);
		// Fiber length
		muscle2->setDefaultFiberLength(0.1);
		muscle1->setDefaultFiberLength(0.1);

		// Initialize the system and get the default state
		State& si = osimModel.initSystem();

		// Define non-zero (defaults are 0) states for the free joint
		CoordinateSet& modelCoordinateSet = osimModel.updCoordinateSet();
		modelCoordinateSet[3].setValue(si, blockSideLength); // set x-translation value
		modelCoordinateSet[3].setSpeedValue(si, 0.1); // set x-speed value
		modelCoordinateSet[4].setValue(si, blockSideLength/2+0.01); // set y-translation value 

		// Compute initial conditions for muscles
		osimModel.equilibrateMuscles(si);

		//osimModel.updMultibodySystem().updMatterSubsystem().setShowDefaultGeometry(false);
		//Visualizer viz(osimModel.getMultibodySystem());
		//osimModel.updMultibodySystem().addEventReporter(new Visualizer::Reporter(viz, 0.1));

		// Create the integrator, force reporter, and manager for the simulation.
		// Create the integrator
		RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
		integrator.setAccuracy(1.0e-3);
		
		// Create the force reporter
		ForceReporter* reporter = new ForceReporter(&osimModel);
		osimModel.addAnalysis(reporter);
		
		// Create the manager
		Manager manager(osimModel, integrator);


		// Print out details of the model
		osimModel.printDetailedInfo(si, cout);

		// Print out the initial position and velocity states
		si.getQ().dump("Initial q's"); // block positions
		si.getU().dump("Initial u's"); // block velocities
		cout << "Initial time: " << si.getTime() << endl;

		// Integrate from initial time to final time
		manager.setInitialTime(initialTime);
		manager.setFinalTime(finalTime);
		cout<<"\n\nIntegrating from "<<initialTime<<" to "<<finalTime<<endl;
		manager.integrate(si);

		//////////////////////////////
		// SAVE THE RESULTS TO FILE //
		//////////////////////////////

		// Save the simulation results
		// Save the states
		Storage statesDegrees(manager.getStateStorage());
		statesDegrees.print("tugOfWar_states.sto");
		osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
		statesDegrees.setWriteSIMMHeader(true);
		statesDegrees.print("tugOfWar_states_degrees.mot");
		// Save the forces
		reporter->getForceStorage().print("tugOfWar_forces.mot");

		// Save the model to a file
		osimModel.print("tugOfWar_model.osim");


		cout << "OpenSim Installation verified.\n" << endl;
		cout << "main() routine time = " << (double)(clock()-start)/CLOCKS_PER_SEC << " seconds" << endl;
	}
    catch (exception ex)
    {
        cout << ex.what() << endl;
		cout << "press any key to continue..." << endl;
		if (waitForUserInput) getchar();
        return 1;
    }
    catch (...)
    {
        cout << "UNRECOGNIZED EXCEPTION" << endl;
 		cout << "press any key to continue..." << endl;
		if (waitForUserInput) getchar();
       return 1;
    }

	cout << "press any key to continue..." << endl;
	if (waitForUserInput) getchar();
	return 0;
}
