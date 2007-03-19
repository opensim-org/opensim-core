// testDisplay1.cpp
// Author:  Ayman Habib
// Tests collection of Visible Objects and their dependencies.
#include <iostream>
#include <string>
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/SIMM/SimmModel.h>
#include <OpenSim/Simulation/SIMM/SimmModelIterator.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmKinematicsEngine.h>

using namespace OpenSim;
using namespace std;


// DECLARATIONS
int TestVisibleObjectCollection();
//_____________________________________________________________________________
/**
 * Test the osimCommon library.
 */
int main(int argc, char* argv[])
{
	cout << "Before loading the model\n";
	SimmModel* model = new SimmModel("C:/Downloads/MODEL/dynamic.xml");
	cout << "After loading the model. Before setup call\n";
	model->setup();
 	cout << "After setup call. Before iterator\n";
    SimmModelIterator *i = new SimmModelIterator(*model);

    SimmBody *gnd = model->getSimmKinematicsEngine().getGroundBodyPtr();
    while (i->getNextBody() != 0) {

        SimmBody *body = i->getCurrentBody();
	}
	cout << "After iterator\n";
	return(0);
}
