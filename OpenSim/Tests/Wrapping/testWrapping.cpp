// testWrapping.cpp
// Author: Jack Middleton
/*
* Copyright (c)  2010, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
// INCLUDE
#include <string>
#include <iostream>
#include <OpenSim/OpenSim.h>


using namespace OpenSim;
using namespace std;

bool equalStorage(Storage& stdStorage, Storage& actualStorage, double tol)
{
    Storage diffStorage(actualStorage);
	diffStorage.subtract(&stdStorage);
	bool equal = true;
	Array<double> dData(-SimTK::Infinity);
	Array<double> sData(-SimTK::Infinity);
	Array<double> aData(-SimTK::Infinity);
	for (int i=0; i <diffStorage.getSize() && equal; i++){
		dData = diffStorage.getStateVector(i)->getData();
		sData = actualStorage.getStateVector(i)->getData();
		aData = stdStorage.getStateVector(i)->getData();
		double dMax = -SimTK::Infinity;
		int worst=-1;
		for (int j=0; j<dData.getSize(); j++){
		    cout << stdStorage.getColumnLabels().get(j+1) << " diff="<< dData[j] << "  std=" <<  sData[j] << "  result=" << aData[j] << endl;
			if (fabs(dData[j]) > dMax) worst = j;
			dMax = std::max(dMax, fabs(dData[j]));
		}
		equal = (dMax <= tol);
		cout << "i, col, colname, diff" << i << ", worst:" << worst <<", name:"<< stdStorage.getColumnLabels().get(worst+1) << ", max="<<dMax << "  tol=" << tol << "   equal=" << equal << endl;
	}
	return equal;
}

int testModel(std::string modelPrefix)
{
	try {

	// CONSTRUCT
	AnalyzeTool analyze(modelPrefix+"_Setup.xml");

	// PRINT MODEL INFORMATION
	Model& model = analyze.getModel();
	cout<<"-----------------------------------------------------------------------\n";
	cout<<"Loaded library\n";
	cout<<"-----------------------------------------------------------------------\n";
	cout<<"-----------------------------------------------------------------------\n\n";

	// RUN
	analyze.run();

	//----------------------------
	// Catch any thrown exceptions
	//----------------------------
	} catch(Exception x) {
		x.print(cout);
		return(-1);
	}
	// Compare results to a standard 
	Storage currentResult("Results/"+modelPrefix+"_Actuation_force.sto");
	Storage stdStorage("std_"+modelPrefix+"_Actuation_force.sto");
	bool equal = equalStorage(stdStorage, currentResult, .1);
	return (equal?0:1);
}

bool simulateModelWithMuscles(const std::string &modelFile)
{
	bool status = true;

	// Create a new OpenSim model
	Model osimModel(modelFile);

	double initialTime = 0;
	double finalTime = 0.5;

	// Define the initial and final control values
	double control = 0.2;

	// Create a prescribed controller that simply applies a function of the force
	PrescribedController actuatorController;
	actuatorController.setActuators(osimModel.updActuators());
	for (int i=0; i<actuatorController.getActuatorSet().getSize(); i++){
		actuatorController.prescribeControlForActuator(i, new Constant(control));
	}

	// add the controller to the model
	osimModel.addController(&actuatorController);

	// Initialize the system and get the state representing the state system
	SimTK::State& si = osimModel.initSystem();
	osimModel.computeEquilibriumForAuxiliaryStates(si); 

	// Create the integrator and manager for the simulation.
	double accuracy = 1.0e-3;
	SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
	integrator.setMaximumStepSize(1);
	integrator.setMinimumStepSize(1.0e-6);
	integrator.setAccuracy(accuracy);
	integrator.setAbsoluteTolerance(1.0e-4);
	Manager manager(osimModel, integrator);

	// Integrate from initial time to final time
	manager.setInitialTime(initialTime);
	manager.setFinalTime(finalTime);
	std::cout<<"\nIntegrating from "<<initialTime<<" to "<<finalTime<<std::endl;
	
	const clock_t start = clock();
	manager.integrate(si);
	std::cout << "simulation time = " << (double)(clock()-start)/CLOCKS_PER_SEC << " seconds\n" << std::endl;

	// Save the simulation results
	Storage states(manager.getStateStorage());
	states.print(osimModel.getName()+"_states.sto");
	osimModel.updSimbodyEngine().convertRadiansToDegrees(states);
	states.setWriteSIMMHeader(true);
	states.print(osimModel.getName()+"_states_degrees.mot");

	osimModel.disownAllComponents();

	return status;
}// end of simulateModelWithSingleMuscle()



int main(int argc,char **argv)
{
	// Baseline perfromance without wrapping
	simulateModelWithMuscles("test_nowrap_vasint.osim");
	// performance with cylnder wrapping
	simulateModelWithMuscles("test_wrapping_vasint.osim");
	// performance with ellipsoid wrapping
	simulateModelWithMuscles("test_wrapEllipsoid_vasint.osim");

	// performance with multiple muscles and no wrapping
	simulateModelWithMuscles("gait2392_pelvisFixed.osim");
	// performance with multiple muscles and wrapping
	simulateModelWithMuscles("Arnold2010_pelvisFixed.osim");

	return(0);
}

