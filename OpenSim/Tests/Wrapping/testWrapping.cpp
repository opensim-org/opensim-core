// testWrapping.cpp
// Author: Ajay Seth
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
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void simulateModelWithMuscles(const string &modelFile, double finalTime=0.5);

int main()
{
	try {
		// Baseline perfromance without wrapping
		simulateModelWithMuscles("test_nowrap_vasint.osim");
		// performance with cylnder wrapping
		simulateModelWithMuscles("test_wrapping_vasint.osim");
		// performance with ellipsoid wrapping
		simulateModelWithMuscles("test_wrapEllipsoid_vasint.osim");
		// performance with multiple muscles and no wrapping
		simulateModelWithMuscles("gait2392_pelvisFixed.osim", 0.1);
		// performance with multiple muscles and wrapping
		simulateModelWithMuscles("Arnold2010_pelvisFixed.osim", 0.1);
		// performance with multiple muscles and wrapping in upper-exremity
		simulateModelWithMuscles("TestShoulderModel.osim", 0.1);
	}
	catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

void simulateModelWithMuscles(const string &modelFile, double finalTime)
{
	// Create a new OpenSim model
	Model osimModel(modelFile);

	double initialTime = 0;

	// Define the initial and final control values
	double control = 0.2;

	// Create a prescribed controller that simply applies a function of the force
	PrescribedController actuatorController;
	actuatorController.setActuators(osimModel.updActuators());
	for (int i=0; i<actuatorController.getActuatorSet().getSize(); i++) {
		actuatorController.prescribeControlForActuator(i, new Constant(control));
	}

	// add the controller to the model
	osimModel.addController(&actuatorController);

	// Initialize the system and get the state representing the state system
	SimTK::State& si = osimModel.initSystem();

	const Set<Muscle>& muscles = osimModel.getMuscles();
	for (int i=0; i<muscles.getSize(); i++){
		muscles[i].setActivation(si, 0.05); //setDisabled(si, true);
	}
	osimModel.equilibrateMuscles(si); 

	osimModel.printBasicInfo(cout);

	// Create the integrator and manager for the simulation.
	double accuracy = 1.0e-4;
	SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
	integrator.setMaximumStepSize(1);
	integrator.setMinimumStepSize(1.0e-9);
	integrator.setAccuracy(accuracy);

	Manager manager(osimModel, integrator);

	// Integrate from initial time to final time
	manager.setInitialTime(initialTime);
	manager.setFinalTime(finalTime);
	cout << "\nIntegrating from " << initialTime << " to " << finalTime << endl;
	
	const clock_t start = clock();
	manager.integrate(si);
	cout << "simulation time = " << (double)(clock()-start)/CLOCKS_PER_SEC << " seconds\n" << endl;

	// Save the simulation results
	Storage states(manager.getStateStorage());
	states.print(osimModel.getName()+"_states.sto");
	osimModel.updSimbodyEngine().convertRadiansToDegrees(states);
	states.setWriteSIMMHeader(true);
	states.print(osimModel.getName()+"_states_degrees.mot");

	osimModel.disownAllComponents();
}// end of simulateModelWithSingleMuscle()