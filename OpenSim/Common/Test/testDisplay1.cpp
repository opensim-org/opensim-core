// testDisplay1.cpp
// Author:  Ayman Habib
// Tests collection of Visible Objects and their dependencies.
#include <iostream>
#include <string>
#include <assert.h>
#include <OpenSim/Simulation/Model/Model.h>

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
	Model* model = new Model("C:/Projects/Models/FullBody/FullBody.xml");
	model->setup();

	delete model;
	return(0);
}
