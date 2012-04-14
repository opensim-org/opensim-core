// testMuscles.cpp
// Author:  Ajay Seth
/*
* Copyright (c) 2005-2011, Stanford University. All rights reserved. 
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
//	testMuscles builds various OpenSim models using the OpenSim API and compares muscle behavior
//  for varying physical parameters (fiber-to-tendon ratio, tendon stiffness, etc...)
//
//	Models tested include:
//      1. PathActuator (Base of Muscle, is controlled tension along a GeometryPath)
//		2. RigidTendonMuscle (Stateless muscle with user-defined fiber f-l, f-v splines)
//      3. Thelen2003Muscle_Deprecated (Simm implementation)
//		4. Thelen2003MuscleV1 (Updated to correspond to the Thelen paper.)
//		4. Thelen2003Muscle (Uses the Muscle interface)
//		5. Schutte1993Muscle(_Deprecated)
//		6. Delp1990Muscle(_Deprecated)
//		
//     Add more test cases to address specific problems with muscle models
//
//==========================================================================================================
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>
#include <OpenSim/Simulation/Model/PathActuator.h>
#include <OpenSim/Simulation/Model/ActuatorWorkMeter.h>
#include <OpenSim/Simulation/Model/JointWorkMeter.h>
#include <OpenSim/Actuators/RigidTendonMuscle.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Analyses/MuscleAnalysis.h>

using namespace OpenSim;
using namespace std;

//==========================================================================================================
static const double accuracy = 1e-4;

// MUSCLE CONSTANTS
static const double maxIsometricForce = 100.0, optimalFiberLength = 0.1, tendonSlackLength = 0.2, pennationAngle = 0.0;
static const double activation = 0.01, deactivation = 0.4,	activation1 = 7.6,	activation2 = 2.5;

void simulateMuscle(PathActuator &aMuscle, const double &startX, const double &act0, 
					Function *motion, 
					Function *control, 
					const double &accuracy);

void testPathActuator();
void testRigidTendonMuscle();
void testThelen2003Muscle_Deprecated();
void testThelen2003Muscle();
void testSchutte1993Muscle();
void testDelp1990Muscle();
void testThelen2003MuscleV1();

int main()
{
	SimTK::Array_<std::string> failures;
    try { testPathActuator();
		cout << "PathActuator Test passed" << endl; }
    catch (const Exception& e)
		{ e.print(cerr); failures.push_back("testPathActuator"); }

    try { testRigidTendonMuscle();
		cout << "RigidTendonMuscle Test passed" << endl; }
    catch (const Exception& e)
		{ e.print(cerr); failures.push_back("testRigidTendonMuscle"); }

    try { testThelen2003Muscle_Deprecated();
		cout << "Thelen2003Muscle_Deprecated Test passed" << endl; }
    catch (const Exception& e)
		{ e.print(cerr); failures.push_back("testThelen2003Muscle_Deprecated"); }

	try { testThelen2003Muscle();
		cout << "Thelen2003Muscle Test passed" << endl; }
    catch (const Exception& e)
		{ e.print(cerr); failures.push_back("testThelen2003Muscle"); }
		
	try { testThelen2003MuscleV1();
		cout << "Thelen2003MuscleV1 Test passed" << endl; }
    catch (const Exception& e)
		{ e.print(cerr); failures.push_back("testThelen2003MuscleV1"); }
	
	/*
	try { testSchutte1993Muscle();
		cout << "Schutte1993Muscle_Deprecated Test passed" << endl; }
    catch (const Exception& e)
		{ e.print(cerr); failures.push_back("testSchutte1993Muscle"); }

	try { testDelp1990Muscle();
		cout << "Delp1990Muscle_Deprecated Test passed" << endl; }
    catch (const Exception& e)
		{ e.print(cerr); failures.push_back("testDelp1990Muscle"); }
	*/

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

	cout << "testMuscles Done" << endl;
    return 0;
}

//==========================================================================================================
// Main test driver to be used on any muscle model (derived from Muscle) so new cases should be easy to add
// Currently, the test only verifies that the work done by the muscle corresponds to the change in system
// energy.
//
// TODO: Test will fail wih prescribe motion until the work done by this constraint is accounted for.
//==========================================================================================================
void simulateMuscle(PathActuator &aMuscle, const double &startX, const double &act0, 
					Function *motion,  // prescribe motion of free end of muscle
					Function *control, // prescribed excitation signal to the muscle
					const double &accuracy)
{
	cout << "\n******************************************************" << endl;
	cout << " Test " << aMuscle.getType() << " Muscle Actuator Type." <<endl;
	cout << "******************************************************" << endl;
	using SimTK::Vec3;

	// Define the initial and final simulation times
	double initialTime = 0.0;
	double finalTime = 1.0;
	
	//Physical properties of the model
	double ballMass = 10;
	double ballRadius = 0.05;
	double anchorWidth = 0.1;

	// Create an OpenSim model and set its name
	Model model;

	// Get a reference to the model's ground body
	Body& ground = model.getGroundBody();
	ground.addDisplayGeometry("box.vtp");
	ground.updDisplayer()->setScaleFactors(Vec3(anchorWidth, anchorWidth, 2*anchorWidth));

	OpenSim::Body ball("ball", ballMass , Vec3(0),  ballMass*SimTK::Inertia::sphere(ballRadius));
	ball.addDisplayGeometry("sphere.vtp");
	ball.updDisplayer()->setScaleFactors(Vec3(2*ballRadius));
	// ball connected  to ground via a slider along X
	double xSinG = optimalFiberLength*cos(pennationAngle)+tendonSlackLength;
	SliderJoint slider("slider", ground, Vec3(anchorWidth/2+xSinG, 0, 0), Vec3(0), ball, Vec3(0), Vec3(0));
	CoordinateSet& jointCoordinateSet = slider.getCoordinateSet();
	jointCoordinateSet[0].setName("tx");
	jointCoordinateSet[0].setDefaultValue(1.0);
	jointCoordinateSet[0].setRangeMin(0); jointCoordinateSet[0].setRangeMax(1.0);
	if(motion != NULL)
		jointCoordinateSet[0].setPrescribedFunction(*motion);
	// add ball to model
	model.addBody(&ball);

	//Attach the muscle
	const string &actuatorType = aMuscle.getType();
	aMuscle.setName("muscle");
	aMuscle.addNewPathPoint("muscle-box", ground, Vec3(anchorWidth/2,0,0));
	aMuscle.addNewPathPoint("muscle-ball", ball, Vec3(-ballRadius,0,0));
	
	ActivationFiberLengthMuscle_Deprecated *aflMuscle = dynamic_cast<ActivationFiberLengthMuscle_Deprecated *>(&aMuscle);
	if(aflMuscle){
		// Define the default states for the muscle that has activation and fiber-length states
		aflMuscle->setDefaultActivation(act0);
		aflMuscle->setDefaultFiberLength(aflMuscle->getOptimalFiberLength());
	}else{
		ActivationFiberLengthMuscle *aflMuscle2 = dynamic_cast<ActivationFiberLengthMuscle *>(&aMuscle);
		if(aflMuscle2){
			// Define the default states for the muscle that has activation and fiber-length states
			aflMuscle2->setDefaultActivation(act0);
			aflMuscle2->setDefaultFiberLength(aflMuscle2->getOptimalFiberLength());
		}
	}


	model.addForce(&aMuscle);

	// Create a prescribed controller that simply applies controls as function of time
	PrescribedController muscleController;
	if(control != NULL){
		muscleController.setActuators(model.updActuators());
		// Set the indiviudal muscle control functions for the prescribed muscle controller
		muscleController.prescribeControlForActuator("muscle", ((Function *)control->copy()));

		// Add the control set controller to the model
		model.addController(&muscleController);
	}

	// Add an energy meter to measure the work done by the muscle actuator 
	ActuatorWorkMeter workMeter(aMuscle, 0.0);
	model.addComponent(&workMeter);

	// Add an energy meter to measure the work done by the joint
	// will be 0 unless joint has prescribed motion
	JointWorkMeter jointWorkMeter(slider, 0.0);
	model.addComponent(&jointWorkMeter);

	// Since all components are allocated on the stack don't have model own them (and try to free)
	model.disownAllComponents();
	model.setName(actuatorType+"ModelTest");
	model.print(actuatorType+"ModelTest.osim");

	// Setup a Muscle Analysis to report all internal values of the muscle during
	// the simulation. If you uncomment, remember to uncomment the corresponding
	// calls to write the results to file after the simualtion.
/*	MuscleAnalysis muscleAnalysis;
	muscleAnalysis.setMuscles(Array<string>("muscle",1));
	model.addAnalysis(&muscleAnalysis);
*/
	// Define visualizer for debugging
	//model.setUseVisualizer(true);

	// Initialize the system and get the default state
	SimTK::State& si = model.initSystem();

	// Define non-zero (defaults are 0) states for the free joint
	CoordinateSet& modelCoordinateSet = model.updCoordinateSet();
	modelCoordinateSet[0].setValue(si, startX); // set x-translation value

	// Check muscle is setup correctly 
	const PathActuator &muscle = dynamic_cast<const PathActuator&>(model.updActuators().get("muscle"));
	double length = muscle.getLength(si);
	double trueLength = startX + xSinG - anchorWidth/2;
	ASSERT_EQUAL(trueLength, length, 0.01*accuracy);

	model.getMultibodySystem().realize(si, SimTK::Stage::Acceleration);
	double Emuscle0 = workMeter.getWork(si);
	//cout << "Muscle initial energy = " << Emuscle0 << endl;
	double Esys0 = model.getMultibodySystem().calcEnergy(si);
	Esys0 += (Emuscle0 + jointWorkMeter.getWork(si));
	//cout << "Total initial system energy = " << Esys0 << endl; 

	// Create the integrator
	SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
	integrator.setAccuracy(accuracy);

	// Create the manager
	Manager manager(model, integrator);

	// Integrate from initial time to final time
	manager.setInitialTime(initialTime);
	manager.setFinalTime(finalTime);
	cout<<"\nIntegrating from " << initialTime<< " to " << finalTime << endl;

	// Start timing the simulation
	const clock_t start = clock();
	// simulate
	manager.integrate(si);

	// how long did it take?
	double comp_time = (double)(clock()-start)/CLOCKS_PER_SEC;

	model.getMultibodySystem().realize(si, SimTK::Stage::Acceleration);
	cout << "Muscle work = " << workMeter.getWork(si) << endl;

	double jointWork = jointWorkMeter.getWork(si);
	double ESysMinusWork = model.getMultibodySystem().calcEnergy(si) - workMeter.getWork(si) - jointWork; 
	cout << "Total system energy - work = " << ESysMinusWork << endl; 
	ASSERT_EQUAL(Esys0, ESysMinusWork, 0.5*accuracy, __FILE__, __LINE__, "System energy with muscle not conserved.");

	// Save the simulation results
	Storage states(manager.getStateStorage());
	states.print(actuatorType+"_states.sto");
	
	//An analysis only writes to a dir that exists, so create here.
	//IO::makeDir("testMuscleResults");
	//muscleAnalysis.printResults(actuatorType, "testMuscleResults");

	// Minimum requirement to pass is simulation of single muscle on slider is real-time
	//ASSERT(comp_time <= (finalTime-initialTime));
	cout << actuatorType << " simulation in " << comp_time << "s, for " << accuracy << " accuracy." << endl;
}


//==========================================================================================================
// Individudal muscle model (derived from Muscle) test cases can be added here
//==========================================================================================================
void testThelen2003Muscle_Deprecated()
{
	Thelen2003Muscle_Deprecated muscle("muscle",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);
	muscle.setActivationTimeConstant(activation);
	muscle.setDeactivationTimeConstant(deactivation);

	double x0 = 0;
	double act0 = 0.2;

	Constant control(0.5);

	Sine motion(0.1, SimTK::Pi, 0);

	simulateMuscle(muscle, x0, act0, NULL, &control, accuracy);
	// Uncomment when work done by prescribed motion constraint is accounted for.
	simulateMuscle(muscle, x0, act0, &motion, &control, accuracy);
}

void testThelen2003Muscle()
{
	Thelen2003Muscle muscle("muscle",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);
	muscle.setActivationTimeConstant(activation);
	muscle.setDeactivationTimeConstant(deactivation);

	double x0 = 0;
	double act0 = 0.2;

	Constant control(0.5);

	Sine motion(0.1, SimTK::Pi, 0);

	simulateMuscle(muscle, x0, act0, NULL, &control, accuracy);
	// Uncomment when work done by prescribed motion constraint is accounted for.
	//simulateMuscle(muscle, x0, act0, &motion, &control, accuracy);
}


void testSchutte1993Muscle()
{
	Schutte1993Muscle_Deprecated muscle("muscle",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);
	muscle.setActivation1(activation1);
	muscle.setActivation2(activation2);

	double x0 = 0;
	double act0 = 0.2;

	Constant control(0.5);

	Sine motion(0.1, SimTK::Pi, 0);

	simulateMuscle(muscle, x0, act0, NULL, &control, accuracy);
}


void testDelp1990Muscle()
{
	Delp1990Muscle_Deprecated muscle("muscle",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);
	muscle.setActivation1(activation1);
	muscle.setActivation2(activation2);
	muscle.setMass(0.1);

	double x0 = 0;
	double act0 = 0.2;

	Constant control(0.5);

	Sine motion(0.1, SimTK::Pi, 0);

	simulateMuscle(muscle, x0, act0, NULL, &control, accuracy);
}

void testPathActuator()
{
	double x0 = 0;
	double act0 = 0.2;
	double loadX = 50;

	PathActuator muscle;
	muscle.setOptimalForce(maxIsometricForce);

	Constant control(0.5);

	Sine motion(0.1, SimTK::Pi, 0);

	simulateMuscle(muscle, x0, act0, NULL, &control, accuracy);
}


void testRigidTendonMuscle()
{
	RigidTendonMuscle muscle("muscle",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);

	double x0 = 0;
	double act0 = 0.2;

	Constant control(0.5);

	Sine motion(0.1, SimTK::Pi, 0);

	simulateMuscle(muscle, x0, act0, NULL, &control, accuracy);
}

void testThelen2003MuscleV1()
{
	double activation = 0.01, deactivation = 0.4;

	Thelen2003MuscleV1 muscle("muscle",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);
	muscle.setActivationTimeConstant(activation);
	muscle.setDeactivationTimeConstant(deactivation);

	double x0 = 0;
	double act0 = 0.2;

	Constant control(0.5);

	Sine motion(0.1, SimTK::Pi, 0);

	simulateMuscle(muscle, x0, act0, NULL, &control, accuracy);
}
