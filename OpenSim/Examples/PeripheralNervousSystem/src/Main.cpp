#include <iostream>
#include <string>
#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/ComponentSet.h>

#include "INIReader.h"

#define TEST_ONLY_GTO

#include "Settings.h"
#include "BlockComponentAnalysis.h"
#include "StepEvent.h"
#include "GolgiTendon.h"

#ifndef TEST_ONLY_GTO
#include "LinGolgiTendon.h"
#include "RenshawCell.h"
#include "SpindleOrgan.h"
#include "Motoneuron.h"
#include "PNSController.h"
#endif


using namespace OpenSim;
using namespace SimTK;
using namespace std;

//globals
INIReader ini;

void test_GTO();

#ifndef TEST_ONLY_GTO
void test_PNS();
void test_LIN_GTO();
void test_RC();
void test_SO();
void test_MN();
#endif

void simulate(Model& model, State& state);

/**
* Main function
*/
int main()
{

	try {
		ini = INIReader(INI_FILE);

		//test_PNS();
		test_GTO();
		//test_LIN_GTO();
		//test_RC();
		//test_SO();
		//test_MN();
	}
	catch (const std::exception& ex)
	{
		cout << "Exception: " << ex.what() << endl;
	}
	catch (...)
	{
		cout << "Unrecognized exception " << endl;
	}
	
	system("pause");
	
	return 0;
}

void test_GTO()
{
	Model model;
	model.buildSystem();//called so I can have access to default subsystem

	GolgiTendon* gto1 = new GolgiTendon("GTO1");
	GolgiTendon* gto2 = new GolgiTendon("GTO2");

	model.addModelComponent(gto1);
	model.addModelComponent(gto2);
	
	BlockComponentAnalysis* custom_analysis = new BlockComponentAnalysis(&model);
	model.addAnalysis(custom_analysis);

	model.buildSystem();//called to finalized

	model.updDefaultSubsystem().addEventHandler(
		new StepEvent(2, 1.0, &model, gto1->getName()));
	model.updDefaultSubsystem().addEventHandler(
		new StepEvent(1, 0.5, &model, gto2->getName()));

	State& state = model.initializeState();

	simulate(model, state);
	custom_analysis->print(BASE_DIR + ini.Get("PATH", "CUSTOM_RESULTS", ""));

}

#ifndef TEST_ONLY_GTO
void test_PNS()
{
	Model model(BASE_DIR + ini.Get("PATH", "MODEL_PATH", ""));
	model.buildSystem();

	PNSController* pns = new PNSController(&model, "pns");
	pns->setAgonistMuscle("muscle1");
	pns->setAntagonistMuscle("muscle2");

	model.addController(pns);

	BlockComponentAnalysis* custom_analysis = new BlockComponentAnalysis(&model);
	model.addAnalysis(custom_analysis);

	Kinematics* kinematics = new Kinematics(&model);
	kinematics->setRecordAccelerations(true);
	kinematics->setInDegrees(true);
	model.addAnalysis(kinematics);

	//MuscleAnalysis* muscle_analysis = new MuscleAnalysis(&model);
	//model.addAnalysis(muscle_analysis);

	model.buildSystem();

	State& state = model.initializeState();

	//init
	CoordinateSet& coordinates = model.updCoordinateSet();
	Coordinate& z_tran = coordinates.get("blockToGround_zTranslation");

	z_tran.setSpeedValue(state, 0.18 * SimTK::Pi);
	model.equilibrateMuscles(state);

	//simulate
	simulate(model, state);

	//results
	custom_analysis->print(BASE_DIR + ini.Get("PATH", "CUSTOM_RESULTS", ""));
	kinematics->printResults(ini.Get("PATH", "KINEMATICS_RESULTS", ""), BASE_DIR);
	//muscle_analysis->printResults(BASE_DIR + ini.Get("PATH", "MUSCLE_RESULTS", ""));
}

void test_LIN_GTO()
{
	Model model;
	model.buildSystem();

	LinGolgiTendon* gto = new LinGolgiTendon("GTO");
	//gto->setInput(State(), 0);
	model.addModelComponent(gto);

//	gto->print("data/lin-gto.xml");

	BlockComponentAnalysis* custom_analysis = new BlockComponentAnalysis(&model);
	model.addAnalysis(custom_analysis);

	model.buildSystem();

	State& state = model.initializeState();
	

	simulate(model, state);
	custom_analysis->print(BASE_DIR + ini.Get("PATH", "CUSTOM_RESULTS", ""));

}

void test_RC()
{
	Model model;
	model.buildSystem();

	RenshawCell* rc = new RenshawCell("RC");
	model.addModelComponent(rc);

	BlockComponentAnalysis* custom_analysis = new BlockComponentAnalysis(&model);
	model.addAnalysis(custom_analysis);

	model.buildSystem();

	State& state = model.initializeState();

	simulate(model, state);
	custom_analysis->print(BASE_DIR + ini.Get("PATH", "CUSTOM_RESULTS", ""));

}

void test_SO()
{
	Model model;
	model.buildSystem();

	SpindleOrgan* so = new SpindleOrgan("SO");
	//so->setGain(1);
	//so->setDelay(0);
	//so->setActivationConstant(1);
	//so->setConstantOne(0.5);
	//so->setConstantTwo(1);
	//so->setSlackLength(1E-3);
	so->setMuscleLength(0.5E-3);
	so->setMuscleVelocity(1);
	model.addModelComponent(so);

	//so->print("data/so.xml");

	BlockComponentAnalysis* custom_analysis = new BlockComponentAnalysis(&model);
	model.addAnalysis(custom_analysis);

	model.buildSystem();

	State& state = model.initializeState();

	simulate(model, state);
	custom_analysis->print(BASE_DIR + ini.Get("PATH", "CUSTOM_RESULTS", ""));
}

void test_MN()
{
	Model model;
	model.buildSystem();

	GolgiTendon* gto = new GolgiTendon("GTO");
	gto->setDelay(0.4);
	model.addModelComponent(gto);

	Array<string> excitatory, inhibitory;
	model.updMiscModelComponentSet().getNames(inhibitory);
	Motoneuron* mn = new Motoneuron("MN", excitatory, inhibitory);
	model.addModelComponent(mn);

	BlockComponentAnalysis* custom_analysis = new BlockComponentAnalysis(&model);
	model.addAnalysis(custom_analysis);

	model.buildSystem();

	State& state = model.initializeState();

	simulate(model, state);
	custom_analysis->print(BASE_DIR + ini.Get("PATH", "CUSTOM_RESULTS", ""));
}
#endif

void simulate(Model& model, State& state)
{
	//setup integrator
	RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
	integrator.setAccuracy(ini.GetReal("SIMULATION", "ACCURACY", 1E-10));
	//integrator.setMaximumStepSize(1E-3);

	//manager
	Manager manager(model, integrator);
	manager.setInitialTime(ini.GetReal("SIMULATION", "START_TIME", 0));
	manager.setFinalTime(ini.GetReal("SIMULATION", "END_TIME", 1));
	
	//integrate
	clock_t begin = clock();
	cout << "Integrating ..." << endl;
	manager.integrate(state);
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "Elapsed time: " << elapsed_secs << endl;
	cout.flush();

	//state results
	OpenSim::Storage state_reporter(manager.getStateStorage());
	model.updSimbodyEngine().convertRadiansToDegrees(state_reporter);
	state_reporter.print(BASE_DIR + ini.Get("PATH", "STATE_RESULTS", ""));
}