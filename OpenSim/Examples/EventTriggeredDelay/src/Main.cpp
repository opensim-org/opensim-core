#include <iostream>
#include <string>

#include <OpenSim/OpenSim.h>

#include "BlockComponentAnalysis.h"
#include "ConstantInput.h"
#include "StepInput.h"
#include "DelaySignal.h"

#include "GolgiTendon.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

#define CUSTOM_RESULTS "custom.sto"

void test_GTO();
void simulate(Model& model, State& state);

/**
* Main function
*/
int main()
{

	try {

		test_GTO();

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

	GolgiTendon* gto1 = new GolgiTendon("GTO1");
	GolgiTendon* gto2 = new GolgiTendon("GTO2");

	model.addModelComponent(gto1);
	model.addModelComponent(gto2);

	BlockComponentAnalysis* custom_analysis = new BlockComponentAnalysis(&model);
	model.addAnalysis(custom_analysis);

	ConstantInput* constant = new ConstantInput(1);
	model.addModelComponent(constant);
	gto1->getInput(BlockComponent::INPUT).connect(constant->getOutput(ConstantInput::OUTPUT));

	StepInput* step = new StepInput(1, 1);
	model.addModelComponent(step);
	//gto2->getInput(BlockComponent::INPUT).connect(step->getOutput(StepInput::OUTPUT));

	DelaySignal* sig = new DelaySignal(0.2, Stage::Dynamics);
	sig->getInput(DelaySignal::INPUT).connect(step->getOutput(StepInput::OUTPUT));
	gto2->getInput(BlockComponent::INPUT).connect(sig->getOutput(DelaySignal::OUTPUT));
	model.addModelComponent(sig);


	model.buildSystem();

	State& state = model.initializeState();

	simulate(model, state);
	custom_analysis->print(CUSTOM_RESULTS);

}

void simulate(Model& model, State& state)
{
	//setup integrator
	RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
	integrator.setAccuracy(1E-10);
	//integrator.setMaximumStepSize(1E-3);

	//manager
	Manager manager(model, integrator);
	manager.setInitialTime(0);
	manager.setFinalTime(2);

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
	//state_reporter.print(BASE_DIR + ini.Get("PATH", "STATE_RESULTS", ""));
}

