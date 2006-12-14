// fallingBlock.cpp
// Author:  Frank C. Anderson

//==============================================================================
//==============================================================================
#include <iostream>
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Simulation/Simm/ActuatorSet.h>
#include <OpenSim/Simulation/Model/ContactForceSet.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Models/Block/rdBlock.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Analyses/Contact.h>
#include <OpenSim/Analyses/Kinematics.h>



using namespace OpenSim;
using namespace std;

//______________________________________________________________________________
/**
 * Run a simulation with a falling block acted on by contact elements
 * and actuators.
 */
int main()
{
	// STEP 1
	// Set output precision
	IO::SetPrecision(8);
	IO::SetDigitsPad(-1);

	// STEP 2
	// Construct the actuator set, contact set, and control set.
	ActuatorSet actuatorSet("fallingBlock_actuators.xml");
	ContactForceSet contactSet("fallingBlock_contacts.xml");
	ControlSet controlSet("fallingBlock_controls.xml");

	// STEP 3
	// Construct the model and print out some information
	// about the model.  The model is build as a dynamically linked library
	// (rdBlock.dll on Windows).
	rdBlock model(&actuatorSet,&contactSet);
	model.printDetailedInfo(cout);

	// STEP 4
	// Alter the initial states if desired.
	// Initial states would normally be specified in a file
	Array<double> yi(0.0,model.getNumStates());
	model.getInitialStates(&yi[0]);
	yi[1] = 0.25;	// Y Position of block center of mass (com)
	yi[7] = 1.0;	// X Velocity of block com
	yi[9] = 0.0;	// Z Velocity of block com
	model.setInitialStates(&yi[0]);

	// STEP 5
	// Set the acceleration due to gravity.
	double g[] = { 0.0, -9.81, 0.0 };
	model.setGravity(g);

	// STEP 6
	// Add analyses to the model.
	// These analyses will gather information during a simulation
	// without altering the simulation.
	// stepInterval specifies how frequently analyses will record info,
	// every four integration steps in this case.
	int stepInterval = 4;
	// Kinematics
	Kinematics *kin = new Kinematics(&model);
	kin->setStepInterval(stepInterval);
	model.addAnalysis(kin);
	// Actuation
	Actuation *actuation = new Actuation(&model);
	actuation->setStepInterval(stepInterval);
	model.addAnalysis(actuation);
	// Contact
	Contact *contact = new Contact(&model);
	contact->setStepInterval(stepInterval);
	model.addAnalysis(contact);


	// STEP 7
	// Construct the integrand and the manager.
	// The model integrand is what is integrated during the simulation.
	// The manager takes care of a variety of low-level initializations.
	ModelIntegrand *integrand = new ModelIntegrand(&model);
	integrand->setControlSet(controlSet);
	Manager manager(integrand);

	// STEP 8
	// Specify the initial and final times of the simulation.
	// In this case, the initial and final times are set based on
	// the range of times over which the controls are available.
	Control *control;
	double ti=0.0,tf=0.25;
	control = controlSet.get("ti");
	if(control!=NULL) ti = control->getControlValue();
	control = controlSet.get("tf");
	if(control!=NULL) tf = control->getControlValue();
	manager.setInitialTime(ti);
	manager.setFinalTime(tf);

	// STEP 9
	// Set up the numerical integrator.
	int maxSteps = 20000;
	IntegRKF *integ = manager.getIntegrator();
	integ->setMaximumNumberOfSteps(maxSteps);
	integ->setMaxDT(1.0e-2);
	integ->setTolerance(1.0e-7);
	integ->setFineTolerance(5.0e-9);

	// STEP 10
	// Integrate
	cout<<"\n\nIntegrating from "<<ti<<" to "<<tf<<endl;
	manager.integrate();

	// STEP 11
	// Print the analysis results.
	model.getAnalysisSet()->printResults("fallingBlock","./");

	return 0;
}
