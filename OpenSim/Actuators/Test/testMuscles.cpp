/* -------------------------------------------------------------------------- *
 *                         OpenSim:  testMuscles.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Matthew Millard                                      *
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

//=============================================================================
//	testMuscles simulates various OpenSim models using the OpenSim API and 
//  compares muscle behavior for varying physical parameters (fiber-to-tendon 
//  ratio, tendon stiffness, etc...)
//
//	Models tested include:
//		1. RigidTendonMuscle (Stateless muscle with user-defined fiber f-l, f-v splines)
//      2. Thelen2003Muscle_Deprecated (Simm implementation)
//		3. Schutte1993Muscle(_Deprecated)
//		4. Delp1990Muscle(_Deprecated)
//		5. Thelen2003Muscle (True to 2003 publication)
//      6. Millard2012EquilibriumMuscle
//      7. Millard2012EquilibriumMuscle
//		
//     Add more test cases to address specific problems with muscle models
//
//==============================================================================
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>
#include <OpenSim/Analyses/osimAnalyses.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

//==============================================================================
static const double IntegrationAccuracy = 1.0e-7;
static const double TestTolerance = 1.0e-5;

// MUSCLE CONSTANTS
static const double MaxIsometricForce0  = 100.0, 
                    OptimalFiberLength0 = 0.1, 
                    TendonSlackLength0  = 0.2, 
                    PennationAngle0     = 0.0,
                    PennationAngle1     = SimTK::Pi/4;

static const double Activation0     = 0.01, 
                    Deactivation0   = 0.4,	
                    ShutteDelpActivation1 = 7.6,	
                    ShutteDelpActivation2 = 2.5;

/*
This function completes a controlled activation, controlled stretch simulation 
of a muscle. After the simulation has completed, the results are
tested in a number of different ways to ensure that the muscle model is 
functioning. These include:
	System Energy test: KE+PE - ActuatorWork = (KE+PE)o, 
	Muscle fiber initializtion: tendon force = (active + passive fiber force) along tendon
    Muscle energy consistency: sum(muscle internal powers) = actuator_power 

@param aMuscle  a path actuator
@param startX   the starting position of the muscle anchor. I have no idea
                why this value is included.
@param act0     the initial activation of the muscle
@param motion   the forced stretch of the simulation
@param control  the activation control signal that is applied to the muscle
@param accuracy the desired accuracy of the integrated solution
@param testTolerance    the desired tolerance associated with the test
@param printResults print the osim model associated with this test.
*/
void simulateMuscle(const Muscle &aMuscle, 
                    double startX, 
                    double act0, 
					const Function *motion, 
					const Function *control, 
					double integrationAccuracy,
                    double testTolerance,
                    bool printResults);

//void testPathActuator();
void testRigidTendonMuscle();
void testThelen2003Muscle_Deprecated();
void testThelen2003Muscle();
void testMillard2012EquilibriumMuscle();
void testMillard2012AccelerationMuscle();
void testSchutte1993Muscle();
void testDelp1990Muscle();

int main()
{
	SimTK::Array_<std::string> failures;

    try { testRigidTendonMuscle();
		cout << "RigidTendonMuscle Test passed" << endl; }
    catch (const Exception& e)
		{ e.print(cerr); failures.push_back("testRigidTendonMuscle"); }
    
    try { testThelen2003Muscle_Deprecated();
		cout << "Thelen2003Muscle_Deprecated Test passed" << endl; }
    catch (const Exception& e)
		{ e.print(cerr); failures.push_back("testThelen2003Muscle_Deprecated");}	
	
	try { testSchutte1993Muscle();
		cout << "Schutte1993Muscle_Deprecated Test passed" << endl; }
    catch (const Exception& e)
		{ e.print(cerr); failures.push_back("testSchutte1993Muscle"); }

	try { testDelp1990Muscle();
		cout << "Delp1990Muscle_Deprecated Test passed" << endl; }
    catch (const Exception& e)
		{ e.print(cerr); failures.push_back("testDelp1990Muscle"); }

    try { testThelen2003Muscle();
		cout << "Thelen2003Muscle Test passed" << endl; }
    catch (const Exception& e)
		{ e.print(cerr); failures.push_back("testThelen2003Muscle"); }
    
    try { testMillard2012EquilibriumMuscle();
		cout << "Millard2012EquilibriumMuscle Test passed" << endl; 
    }catch (const Exception& e){ 
        e.print(cerr);
        failures.push_back("testMillard2012EquilibriumMuscle");
    }
    try { testMillard2012AccelerationMuscle();
		cout << "Millard2012AccelerationMuscle Test passed" << endl; 
    }catch (const Exception& e){ 
        e.print(cerr);
        failures.push_back("testMillard2012AccelerationMuscle");
    }

    cout <<"************************************************************"<<endl;
    cout <<"************************************************************"<<endl;

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    
	cout << "testMuscles Done" << endl;
    return 0;
}


void testMuscle(const Muscle& muscleModel)
{
	double act0 = 0.2;
	Constant control(act0);

	Sine motion(OptimalFiberLength0/2, 2*SimTK::Pi, SimTK::Pi/2.0);
	double x0 = motion.calcValue(SimTK::Vector(1, 0.0));

	// Simulate with the position of the ball free to move according to
	// applied muscl force
	simulateMuscle(muscleModel, x0, act0, NULL, &control,
		IntegrationAccuracy, TestTolerance, false);

	// Simulate with the position of the ball prescribed as a sinusoid
	simulateMuscle(muscleModel, x0, act0, &motion, &control, 
				   IntegrationAccuracy, TestTolerance, false);
}

//==============================================================================
// Individudal muscle model (derived from Muscle) test cases can be added here
//==============================================================================
void testRigidTendonMuscle()
{
	RigidTendonMuscle muscle("muscle",
                              MaxIsometricForce0,
                              OptimalFiberLength0,
                              TendonSlackLength0,
                              PennationAngle0);
	testMuscle(muscle);
}



void testThelen2003Muscle_Deprecated()
{
	Thelen2003Muscle_Deprecated muscle("muscle",
                                        MaxIsometricForce0,
                                        OptimalFiberLength0,
                                        TendonSlackLength0,
                                        PennationAngle0);

	muscle.setActivationTimeConstant(Activation0);
	muscle.setDeactivationTimeConstant(Deactivation0);

	testMuscle(muscle);
	
}

void testThelen2003Muscle()
{

	Thelen2003Muscle muscle0("muscle",
                            MaxIsometricForce0,
                            OptimalFiberLength0,
                            TendonSlackLength0,
                            PennationAngle0);
	muscle0.setActivationTimeConstant(Activation0);
	muscle0.setDeactivationTimeConstant(Deactivation0);

	Thelen2003Muscle muscle1(muscle0);
	muscle1.setPennationAngleAtOptimalFiberLength(PennationAngle1);

	testMuscle(muscle0);

	cout << "Now test Thelen2003Muscle test with pennation = ";
	cout << SimTK::convertRadiansToDegrees(PennationAngle1) << "degrees." << endl;
    
	testMuscle(muscle1);
}


void testMillard2012EquilibriumMuscle()
{
	Millard2012EquilibriumMuscle muscle("muscle",
                            MaxIsometricForce0,
                            OptimalFiberLength0,
                            TendonSlackLength0,
                            PennationAngle0);

    muscle.setActivationTimeConstant(Activation0);
    muscle.setDeactivationTimeConstant(Deactivation0);

	testMuscle(muscle);
}

void testMillard2012AccelerationMuscle()
{
	Millard2012AccelerationMuscle muscle("muscle",
                            MaxIsometricForce0,
                            OptimalFiberLength0,
                            TendonSlackLength0,
                            PennationAngle0);

    //otherwise the simulations are a bit slow ...
    muscle.setMass(muscle.getMass()*10);

    MuscleFirstOrderActivationDynamicModel actMdl = muscle.getActivationModel();
    actMdl.setActivationTimeConstant(Activation0);
    actMdl.setDeactivationTimeConstant(Deactivation0);
    muscle.setActivationModel(actMdl);

	testMuscle(muscle);
}

void testSchutte1993Muscle()
{
	Schutte1993Muscle_Deprecated muscle("muscle",
                                        MaxIsometricForce0,
                                        OptimalFiberLength0,
                                        TendonSlackLength0,
                                        PennationAngle0);

	muscle.setActivation1(ShutteDelpActivation1);
	muscle.setActivation2(ShutteDelpActivation2);

	testMuscle(muscle);
}

void testDelp1990Muscle()
{
	Delp1990Muscle_Deprecated muscle("muscle",
                                    MaxIsometricForce0,
                                    OptimalFiberLength0,
                                    TendonSlackLength0,
                                    PennationAngle0);

	muscle.setActivation1(ShutteDelpActivation1);
	muscle.setActivation2(ShutteDelpActivation2);
	muscle.setMass(0.1);

	testMuscle(muscle);
}

/*==============================================================================
Main test driver to be used on any muscle model (derived from Muscle) so new
cases should be easy to add. The test only verifies that the work done
by the muscle actuator corresponds to the change in system energy. It also
checks that the consituent powers of the Muscle sum to the actuator power.
==============================================================================*/
void simulateMuscle(
	const Muscle &aMuscModel,
	double startX,
	double act0,
	const Function *motion,  // prescribe motion of free end of muscle
	const Function *control, // prescribed excitation signal to the muscle
	double integrationAccuracy,
	double testTolerance,
	bool printResults)
{
	string prescribed = (motion == NULL) ? "" : " (Prescribed Motion)";

	cout << "**********************************************************" << endl;
	cout << " Test " << aMuscModel.getConcreteClassName() << prescribed << endl;
	cout << "**********************************************************" << endl;
	using SimTK::Vec3;

	//==========================================================================
	// 0. SIMULATION SETUP: Create the ball translating in ground
	//==========================================================================

	// Define the initial and final simulation times
	double initialTime = 0.0;
	double finalTime = 1.0;

	//Physical properties of the model
	double ballMass = 100;
	double ballRadius = 0.05;
	double anchorWidth = 0.1;

	// Create an OpenSim model
	Model model;

	double optimalFiberLength = aMuscModel.getOptimalFiberLength();
	double pennationAngle = aMuscModel.getPennationAngleAtOptimalFiberLength();
	double tendonSlackLength = aMuscModel.getTendonSlackLength();

	// Use a copy of the muscle model passed in to add path points later
	PathActuator *aMuscle = aMuscModel.clone();

	// Get a reference to the model's ground body
	Body& ground = model.getGroundBody();
	ground.addDisplayGeometry("box.vtp");
	ground.updDisplayer()
		->setScaleFactors(Vec3(anchorWidth, anchorWidth, 2 * anchorWidth));

	OpenSim::Body * ball = new OpenSim::Body("ball",
		ballMass,
		Vec3(0),
		ballMass*SimTK::Inertia::sphere(ballRadius));

	ball->addDisplayGeometry("sphere.vtp");
	ball->updDisplayer()->setScaleFactors(Vec3(2 * ballRadius));
	// ball connected  to ground via a slider along X
	double xSinG = optimalFiberLength*cos(pennationAngle) + tendonSlackLength;

	SliderJoint slider("slider",
		ground,
		Vec3(anchorWidth / 2 + xSinG, 0, 0),
		Vec3(0),
		*ball,
		Vec3(0),
		Vec3(0));

	CoordinateSet& jointCoordinateSet = slider.upd_CoordinateSet();
	jointCoordinateSet[0].setName("tx");
	jointCoordinateSet[0].setDefaultValue(startX);
	jointCoordinateSet[0].setRangeMin(0);
	jointCoordinateSet[0].setRangeMax(1.0);

	if (motion != NULL){
		jointCoordinateSet[0].setPrescribedFunction(*motion);
		jointCoordinateSet[0].setDefaultIsPrescribed(true);
	}
	// add ball to model
	model.addBody(ball);

	//==========================================================================
	// 1. SIMULATION SETUP: Add the muscle
	//==========================================================================
	//Attach the muscle
	const string &actuatorType = aMuscle->getConcreteClassName();
	aMuscle->setName("muscle");
	aMuscle->addNewPathPoint("muscle-box", ground, Vec3(anchorWidth / 2, 0, 0));
	aMuscle->addNewPathPoint("muscle-ball", *ball, Vec3(-ballRadius, 0, 0));

	ActivationFiberLengthMuscle_Deprecated *aflMuscle
		= dynamic_cast<ActivationFiberLengthMuscle_Deprecated *>(aMuscle);
	if (aflMuscle){
		// Define the default states for the muscle that has 
		//activation and fiber-length states
		aflMuscle->setDefaultActivation(act0);
		aflMuscle->setDefaultFiberLength(aflMuscle->getOptimalFiberLength());
	}
	else{
		ActivationFiberLengthMuscle *aflMuscle2
			= dynamic_cast<ActivationFiberLengthMuscle *>(aMuscle);
		if (aflMuscle2){
			// Define the default states for the muscle 
			//that has activation and fiber-length states
			aflMuscle2->setDefaultActivation(act0);
			aflMuscle2->setDefaultFiberLength(aflMuscle2
				->getOptimalFiberLength());
		}
	}

	model.addForce(aMuscle);

	// Create a prescribed controller that simply 
	//applies controls as function of time
	PrescribedController * muscleController = new PrescribedController();
	if (control != NULL){
		muscleController->setActuators(model.updActuators());
		// Set the indiviudal muscle control functions 
		//for the prescribed muscle controller
		muscleController->prescribeControlForActuator("muscle", control->clone());

		// Add the control set controller to the model
		model.addController(muscleController);
	}

	// Set names for muscles / joints.
	Array<string> muscNames;
	muscNames.append(aMuscle->getName());
	Array<string> jointNames;
	jointNames.append("slider");

	//==========================================================================
	// 2. SIMULATION SETUP: Instrument the test with probes
	//==========================================================================

	// Add an ActuatorPowerProbe to measure the work done by the muscle actuator 
	ActuatorPowerProbe * actWorkProbe = new ActuatorPowerProbe(muscNames, true, 1);
	actWorkProbe->setOperation("integrate");
	actWorkProbe->setInitialConditions(SimTK::Vector(1, 0.0));
	model.addProbe(actWorkProbe);

	// Add a JointInternalPowerProbe to measure the work done by the joint
	// will be 0 unless joint has prescribed motion
	JointInternalPowerProbe * jointWorkProbe = new JointInternalPowerProbe(jointNames, true, 1);
	jointWorkProbe->setOperation("integrate");
	jointWorkProbe->setInitialConditions(SimTK::Vector(1, 0.0));
	model.addProbe(jointWorkProbe);

	/* Since all components are allocated on the stack don't have model
	own them (and try to free)*/
	//	model.disownAllComponents();
	model.setName(actuatorType + "ModelTest");
	model.print(actuatorType + "ModelTest.osim");

	/* Setup a Muscle Analysis to report all internal values of the
	muscle during the simulation. If you uncomment, remember to
	uncomment the corresponding calls to write the results to
	file after the simualtion.*/
	string muscClassName = aMuscModel.getConcreteClassName();
	bool isDeprecated = (muscClassName.find("Deprecated") < muscClassName.length());
	if (!isDeprecated){
		MuscleAnalysis * muscleAnalysis = new MuscleAnalysis();
		Array<string> tmp;
		tmp.append("muscle");
		muscleAnalysis->setMuscles(tmp);
		model.addAnalysis(muscleAnalysis);
	}

	// Define visualizer for debugging
	//model.setUseVisualizer(true);

	//==========================================================================
	// 3. SIMULATION Initialization
	//==========================================================================

	// Initialize the system and get the default state    
	SimTK::State& si = model.initSystem();
	model.getMultibodySystem().realize(si, SimTK::Stage::Dynamics);

	// Define non-zero (defaults are 0) states for the slider joint
	// set x-translation value
	CoordinateSet& modelCoordinateSet = model.updCoordinateSet();
	modelCoordinateSet[0].setValue(si, startX, true);

	// find muscle fiber length where tension in tendon and fiber are
	// in equiibrium given the current configuration and activation
	// of the whole muscle
	model.equilibrateMuscles(si);

	//Copy the initial state
	SimTK::State initialState(si);

	// Check muscle is setup correctly 
	const Muscle &muscle = dynamic_cast<const Muscle&>(*aMuscle);
	double length = muscle.getLength(si);
	double trueLength = startX + xSinG - anchorWidth / 2;

	ASSERT_EQUAL(length / trueLength, 1.0, testTolerance, __FILE__, __LINE__,
		"testMuscles: path failed to initialize to correct length.");

	/*=========================================================================
	4. Initial Equlibrium test:

	Ft   : the force in the tendon
	aFm_t: the active muscle fiber force along the tendon
	pFm_t: the passive muscle fiber force along the tendon

	Verify: Ft - (aFm_t + pFm_t) = 0 at initial time.
	============================================================================*/
	try{
		double Ft = muscle.getTendonForce(si);
		double aFm_t = muscle.getActiveFiberForceAlongTendon(si);
		double pFm_t = muscle.getPassiveFiberForceAlongTendon(si);
		cout << "Ft = " << Ft << " :: aFm_t + pFm_t = " << aFm_t + pFm_t << endl;
		ASSERT_EQUAL(Ft, aFm_t + pFm_t, testTolerance, __FILE__, __LINE__,
			"testMuscles: Initial tendon-fiber equilibriun NOT achieved.");
	}
	catch (const std::exception& x){
		cout << x.what() << endl;
		// if a Delp deprecated muscle just skip it since it should not be used
		// anyways
		if (isDeprecated && (muscClassName.find("Delp") < muscClassName.length())){
			cout << muscClassName << " does not compute fiber forces." << endl;
			cout << "Ignore initial equilibrium condition test." << endl;
		}
		else { // every other muscle throw rethrow the exception to fail the test
			throw x;
		}
	}
	//==========================================================================

	model.getMultibodySystem().realize(si, SimTK::Stage::Acceleration);

	double Esys0 = model.getMultibodySystem().calcEnergy(si);
	double KEsys0 = model.getMultibodySystem().calcKineticEnergy(si);
	double PEsys0 = model.getMultibodySystem().calcPotentialEnergy(si);
	double muscPEo = muscle.getMusclePotentialEnergy(si);
	//cout << "Total initial system energy = " << Esys0 << endl; 

	//==========================================================================
	// 5. SIMULATION Integration
	//==========================================================================
	// Create the integrator
	SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
	integrator.setAccuracy(integrationAccuracy);

	// Create the manager
	Manager manager(model, integrator);

	// Integrate from initial time to final time
	manager.setInitialTime(initialTime);
	manager.setFinalTime(finalTime);

	// Start timing the simulation
	const clock_t start = clock();
	// simulate
	manager.integrate(si);

	// how long did it take?
	double comp_time = (double)(clock() - start) / CLOCKS_PER_SEC;

	// Save the simulation results
	Storage states(manager.getStateStorage());
	states.print(actuatorType + "_states.sto");

	//==========================================================================
	// 6. SIMULATION Reporting
	//==========================================================================
	cout << "Simulation time: " << finalTime - initialTime << "s |";
	cout << "Computation time: " << comp_time << "s" << endl;

	//An analysis only writes to a dir that exists, so create here.
	if (printResults == true){
		IO::makeDir("testMuscleResults");
		model.getAnalysisSet()[0]
			.printResults(actuatorType, "testMuscleResults");
	}

	double actuatorWork = actWorkProbe->getProbeOutputs(si)(0);
	cout << "Actuator work = " << actuatorWork << endl;

	//==========================================================================
	// 7. SIMULATION Tests
	//==========================================================================
	/*==========================================================================
	7a. System energy test:  KE+PE-W = const ?

	Check that system energy minus the work done by the muscle is conserved.
	This test is independent of the actuator type and its internal engergy
	storage since we are asking for the actuator's contribution on the
	system. A more detailed test (to follow) verifies that the energy flow
	(power transimition) within a muscle is conserved.
	In this case we must subtract the muscle's internal potential energy
	storage in the fiber and tenson elastic elements, because we are not
	accounting for the total fiber work, just the actuator work.
	============================================================================*/
	model.getMultibodySystem().realize(si, SimTK::Stage::Acceleration);
	double Esys = model.getMultibodySystem().calcEnergy(si);
	double KEsys = model.getMultibodySystem().calcKineticEnergy(si);
	double xSpeed = modelCoordinateSet[0].getSpeedValue(si);
	double KEsysCheck = 0.5*ballMass*xSpeed*xSpeed;
	double PEsys = model.getMultibodySystem().calcPotentialEnergy(si);
	double muscPE = dynamic_cast<const Muscle*>(&muscle)->getMusclePotentialEnergy(si);
	double jointWork = jointWorkProbe->getProbeOutputs(si)[0];
	cout << "Joint work (prescribe motion) = " << jointWork << endl;

	// Total system energy - work done by the actuator and by the prescribed
	// motion that is part of the work done by the joint and its constraints.
	// We must subtract the change in potential energy of the muscle, since we 
	// are only looking at the work the actuator does on the system and not
	// how internal structures store/transfer energy to achieve the forces
	// that is applies to the multibody system.
	double deltaPEmusc = muscPE - muscPEo;
	double ESysMinusWork = Esys - actuatorWork - jointWork - deltaPEmusc;

	cout << "Esys - Work = " << ESysMinusWork << " :: Esys0 = " << Esys0 << endl;
	ASSERT_EQUAL(ESysMinusWork, Esys0, testTolerance, __FILE__, __LINE__,
		"testMuscles: System_Energy - Work -not conserved.");

	/*==========================================================================
	7b. Internal consistency test:
	muscle(Actuator)Pwr = tendonPwr + fiberActivePwr + fiberPassivePwr
	============================================================================*/
	if (!isDeprecated){
		const MuscleAnalysis* muscleAnalysis =
			dynamic_cast<const MuscleAnalysis*>(&model.getAnalysisSet()[0]);
		Storage *fiberActivePwrSto
			= muscleAnalysis->getFiberActivePowerStorage();
		Storage *fiberPassivePwrSto
			= muscleAnalysis->getFiberPassivePowerStorage();
		Storage *tendonPwrSto
			= muscleAnalysis->getTendonPowerStorage();
		Storage *musclePwrSto
			= muscleAnalysis->getMusclePowerStorage();

		double *fiberActivePwrDat = NULL;
		double *fiberPassivePwrDat = NULL;
		double *tendonPwrDat = NULL;
		double *musclePwrDat = NULL;

		fiberActivePwrSto->getDataColumn("#1", fiberActivePwrDat);
		fiberPassivePwrSto->getDataColumn("#1", fiberPassivePwrDat);
		tendonPwrSto->getDataColumn("#1", tendonPwrDat);
		musclePwrSto->getDataColumn("#1", musclePwrDat);

		int numSteps = fiberActivePwrSto->getSize();
		double sumOfInternalPowers = 0;

		for (int i = 0; i<numSteps; i++){
			sumOfInternalPowers = tendonPwrDat[i] +
				fiberActivePwrDat[i] + fiberPassivePwrDat[i];

			ASSERT_EQUAL(sumOfInternalPowers, musclePwrDat[i], testTolerance,
				__FILE__, __LINE__,
				"FAILED internal consistency test. ");

		}
		cout << "PASSED internal consistency test:" << endl;
		cout << "       tendon_pwr + fiberActive_pwr + fiberPassive+pwr " << endl;
		cout << "       = muscle_pwr, within accuracy of " << testTolerance << endl;
	}
} // END of simulateMuscle()
