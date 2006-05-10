// fallingBlockSimple.cpp
// author: Frank C. Anderson
#include <stdlib.h>
#include <stdio.h>
#include <RD/Tools/IO.h>
#include <RD/Simulation/Model/Model.h>
#include <RD/Models/Block/rdBlock.h>
#include <RD/Simulation/Manager/Manager.h>

/**  Some points:
  1. Once a model is developed, this is all the source code that is
     needed- most classes are loaded in dynamically-linked libraries.
  2. Model initialization happens on 1 line.
  3. The Manager, which can be side-stepped if desired, 
     takes care of the busy work to construct the integrator etc.
  4. By default, an integration is guarranteed to store the controls,
     states, and pseudostates.  These can be printed out, and read back
	 in subsequent simulations.
 */

//______________________________________________________________________________
/**
 * Run a simple simulation of a falling block.
 */
int main(int argc,char **argv)
{
	// CONSTRUCT MODEL
	rdBlock *model = new rdBlock();

	// PRINT SOME MODEL INFORMATION
	double mass = model->getMass(0);
	int nx = model->getNX();
	int ny = model->getNY();
	int nyp = model->getNYP();
	printf("mass = %lf\n",mass);
	printf("model has %d controls, %d states, and %d pseudo-states.\n",
		nx,ny,nyp);

	// CONSTRUCT SIMULATION MANAGER
	double ti = 0.0;
	double tf = 1.0;
	Manager *manager = new Manager(model);
	manager->setInitialTime(ti);
	manager->setFinalTime(tf);

	// INTEGRATE
	manager->integrate();

	// PRINT STATES
	Storage *yStore = manager->getIntegrator()->getStateStorage();
	yStore->print("Data/fallingBlockSimple_states.sto");

	// CLEANUP
	delete model;
	delete manager;
	delete yStore;

	return(0);
}
