// testSimbodyEngine.cpp
// Author:  Frank C. Anderson

//==============================================================================
//==============================================================================
#include <iostream>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Simulation/Model/ActuatorSet.h>
#include <OpenSim/Simulation/Model/ContactForceSet.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/PointKinematics.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/GeneralizedForce.h>
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/SimbodyEngine.h>

//#include <OpenSim/Simulation/TestSuite/rdModelTestSuite.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//______________________________________________________________________________
/**
 * Run a simulation of a pendulum with Simbody as the underlying dynamics
 * engine.
 */
int main()
{
	// STEP 1
	// Set output precision
	IO::SetPrecision(20);
	//IO::SetDigitsPad(-1);

	// STEP 2
	// Construct the actuator set, contact set, and control set.
	//ActuatorSet actuatorSet("fallingBlock_actuators.xml");
	//ContactForceSet contactSet("fallingBlock_contacts.xml");
	//ControlSet controlSet("fallingBlock_controls.xml");

	// STEP 3
	// Construct the model and print out some information
	// about the model.  The model is build as a dynamically linked library
	// (rdBlock.dll on Windows).
	SimbodyEngine *pendulum = new SimbodyEngine();
	//Model *model = new Model;
	//model->setDynamicsEngine(*pendulum);
	Model *model;
	try {
		model = new Model("pendulum.osim");
		model->setup();
	} catch(OpenSim::Exception x) {
		x.print(cerr);
		return -1;
	}

	// ADD FORCE(S)
	int index = 0;
	// Gen. Force 1
	GeneralizedForce *gf1 = new GeneralizedForce(model->getDynamicsEngine().getCoordinateSet()->get(index)->getName());
	gf1->setName("gf2");
	gf1->setModel(model);
	gf1->setOptimalForce(15);
	// Gen. Force 2
	GeneralizedForce *gf2 = new GeneralizedForce(model->getDynamicsEngine().getCoordinateSet()->get(index)->getName());
	gf2->setName("gf2");
	gf2->setModel(model);
	gf2->setOptimalForce(20);
	// Force 1
	Force *f1 = new Force("Pendulum","ground");
	OpenSim::Array<double> directionA(0.0,3);  directionA[0] = 1.0;
	f1->setName("f1");
	f1->setForceDirectionA(&directionA[0]);
	OpenSim::Array<double> massCenter(0.0,3);
	model->getDynamicsEngine().getBodySet()->get("Pendulum")->getMassCenter(&massCenter[0]);
	cout<<"mass center = "<<massCenter<<endl;
	f1->setPointB(&massCenter[0]);
	f1->setOptimalForce(30.210);
	f1->setModel(model);
	// Add to model
	//model->getActuatorSet()->append(gf1);
	//model->getActuatorSet()->append(gf2);
	//model->getActuatorSet()->append(f1);

	// Setup model
	model->setup();
	OpenSim::Array<double> inertia(0.0,9);
	model->getDynamicsEngine().getBodySet()->get("ground")->setInertia(inertia);
	model->printDetailedInfo(cout);

	// STEP 4
	// Alter the initial states if desired.
	// Initial states would normally be specified in a file
	OpenSim::Array<double> yi(0.0,model->getNumStates());
	model->getInitialStates(&yi[0]);
	yi[0] = 0.1;	// Initial angle of pendulum from vertical
	model->setInitialStates(&yi[0]);

	// STEP 5
	// Set the acceleration due to gravity.
	double g[] = { 0.0, -10.0, 0.0 };
	model->setGravity(g);

	// STEP 6
	// Add analyses to the model->
	// These analyses will gather information during a simulation
	// without altering the simulation.
	// stepInterval specifies how frequently analyses will record info,
	// every four integration steps in this case.
	int stepInterval = 1;
	// Kinematics
	Kinematics *kin = new Kinematics(model);
	kin->setStepInterval(stepInterval);
	model->addAnalysis(kin);
	// Point Kinematics
	OpenSim::Array<double> point(0.0,3);
	BodySet *bodySet = model->getDynamicsEngine().getBodySet();
	int nb = bodySet->getSize();
	for(int i=0;i<nb;i++) {
		PointKinematics *pointKin = new PointKinematics(model);
		AbstractBody *body = bodySet->get(i);
		pointKin->setBodyPoint(body->getName(),&point[0]);
		pointKin->setPointName(body->getName().c_str());
		model->addAnalysis(pointKin);
	}
	// Point Kinematics
	//PointKinematics *pointKin2 = new PointKinematics(model);
	//pointKin2->setBodyPoint("Pendulum2",&point[0]);
	//pointKin2->setPointName("Pendulum2Frame");
	//model->addAnalysis(pointKin2);
	// Actuation
	Actuation *actuation = new Actuation(model);
	actuation->setStepInterval(stepInterval);
	model->addAnalysis(actuation);
	// Contact
	//Contact *contact = new Contact(&model);
	//contact->setStepInterval(stepInterval);
	//model->addAnalysis(contact);


	// STEP 7
	// Construct the integrand and the manager.
	// The model integrand is what is integrated during the simulation.
	// The manager takes care of a variety of low-level initializations.
	ModelIntegrand *integrand = new ModelIntegrand(model);
	//integrand->setControlSet(controlSet);
	int nx = integrand->getControlSet()->getSize();
	cout<<"NumControls = "<<nx<<endl;
	for(int i=0;i<nx;i++) {
		Control *x = integrand->getControlSet()->get(i);
		if(x) x->setControlValue(0,1.0);
	}
	Manager manager(integrand);

	// STEP 8
	// Specify the initial and final times of the simulation.
	// In this case, the initial and final times are set based on
	// the range of times over which the controls are available.
	//Control *control;
	double ti=0.0,tf=10.0;
	//control = controlSet.get("ti");
	//if(control!=NULL) ti = control->getControlValue();
	//control = controlSet.get("tf");
	//if(control!=NULL) tf = control->getControlValue();
	manager.setInitialTime(ti);
	manager.setFinalTime(tf);

	// STEP 9
	// Set up the numerical integrator.
	int maxSteps = 20000;
	IntegRKF *integ = manager.getIntegrator();
	integ->setMaximumNumberOfSteps(maxSteps);
	integ->setMaxDT(1.0e-1);
	integ->setTolerance(1.0e-7);
	integ->setFineTolerance(5.0e-9);

	// STEP 10
	// Integrate
	cout<<"\n\nIntegrating from "<<ti<<" to "<<tf<<endl;
	manager.integrate();

	// STEP 11
	// Print the analysis results.
	model->getAnalysisSet()->printResults("SimbodyPendulum","./");
	//model->print("pendulum.osim");

//	ModelTestSuite aModelTestSuite;
//	aModelTestSuite.Test(model);


	// DELETE MODEL
//	delete model;

	return 0;
}
