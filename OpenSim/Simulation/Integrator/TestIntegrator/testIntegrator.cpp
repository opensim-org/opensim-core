// testIntegrator.cpp
// Author:  Frank C. Anderson


// INCLUDES
#include <iostream>
#include <string>
#include <iostream>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Integrator/IntegRKF.h>
#include <OpenSim/Simulation/Integrator/Integrand.h>
#include <OpenSim/Simulation/integrator/TestIntegrator/SampleIntegrand.h>



using namespace OpenSim;
using namespace std;


//_____________________________________________________________________________
/**
 * Test the integrator classes.
 */
int main(int argc, char* argv[])
{
	// CONSTRUCT INTEGRATOR
	SampleIntegrand integrand;
	IntegRKF rkf(&integrand);
	rkf.setTolerance(1.0e-8);
	rkf.setMaximumNumberOfSteps(10000);

	// STORAGE
	Storage *yStore = new Storage();
	yStore->setName("SampleIntegrand");
	integrand.setStateStorage(yStore);

	// CONSTRUCT STATES
	Array<double> y(0.0);
	y.setSize(integrand.getSize());

	// INTEGRATE
	rkf.integrate(0.0,1.0,&y[0],1.0e-8);

	// PRINT RESULTS
	cout<<"\n\nResults:\n";
	cout<< y <<endl;
	yStore->print("sampleIntegrand.sto");

	// CLEANUP
	delete yStore;

	return(0);
}


