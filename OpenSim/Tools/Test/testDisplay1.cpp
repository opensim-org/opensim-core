// testDisplay1.cpp
// Author:  Ayman Habib
// Tests collection of Visible Objects and their dependencies.
#include <iostream>
#include <string>
#include <assert.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/Geometry.h>
#include <OpenSim/Simulation/SIMM/SimmModel.h>
#include <OpenSim/Simulation/SIMM/SimmKinematicsEngine.h>
#include <OpenSim/Simulation/SIMM/SimmModelVisibleIterator.h>
#include <OpenSim/Simulation/SIMM/SimmModelIterator.h>
#include <OpenSim/Simulation/SIMM/BodyIterator.h>

using namespace OpenSim;
using namespace std;


// DECLARATIONS
int TestVisibleObjectCollection();
//_____________________________________________________________________________
/**
 * Test the rdTools library.
 */
int main(int argc, char* argv[])
{
	AbstractModel* model = new AbstractModel("dynamic.xml");
	model->setup();

	BodyIterator iter = model->getDynamicsEngine().newBodyIterator();
	while(!iter.finished()){
		AbstractBody& nextBody = iter.getCurrent();
		
		cout << "Object:" << (owner?owner->getName():"no-owner") << "\t"  
			<< "#geo files+analytical" << (nextVisibleObject->countGeometry()) << "\t" 
			<< "Dependents" << nextVisibleObject->countDependents()
			<< endl;
		iter++;
	}
	return(0);
}
