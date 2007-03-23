// Written by Eran Guendelman, March 2007

#include <string>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/LoadModel.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <OpenSim/Simulation/Model/BodySet.h>

using namespace OpenSim;
using namespace std;

int main(int argc,char **argv)
{
	LoadOpenSimLibrary("osimActuators");
	LoadOpenSimLibrary("osimSdfastEngine");

	cout << endl << endl;
	Model *model1 = new Model("model1.osim");
	model1->setup();

	cout << endl << endl;
	Model *model2 = new Model("model2.osim");
	model2->setup();

	cout << endl << endl;

	int bodyidx = 1;

	cout << "Mass of model1:body1 = " << model1->getDynamicsEngine().getBodySet()->get(bodyidx)->getMass() << endl;
	cout << "Mass of model2:body1 = " << model2->getDynamicsEngine().getBodySet()->get(bodyidx)->getMass() << endl;

	cout << "\nSetting model1:body1 mass to 12345" << endl;
	model1->getDynamicsEngine().getBodySet()->get(bodyidx)->setMass(12345);

	cout << "Mass of model1:body1 = " << model1->getDynamicsEngine().getBodySet()->get(bodyidx)->getMass() << endl;
	cout << "Mass of model2:body1 = " << model2->getDynamicsEngine().getBodySet()->get(bodyidx)->getMass() << endl;

	cout << "\nSetting model2:body1 mass to 54321" << endl;
	model2->getDynamicsEngine().getBodySet()->get(bodyidx)->setMass(54321);

	cout << "Mass of model1:body1 = " << model1->getDynamicsEngine().getBodySet()->get(bodyidx)->getMass() << endl;
	cout << "Mass of model2:body1 = " << model2->getDynamicsEngine().getBodySet()->get(bodyidx)->getMass() << endl;
}
