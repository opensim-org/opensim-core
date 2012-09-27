/* -------------------------------------------------------------------------- *
 *                         OpenSim:  testWrapping.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
// INCLUDE
#include <OpenSim/OpenSim.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

#include <string>
#include <iostream>

using namespace OpenSim;
using namespace std;

void simulateModelWithMuscles(const string &modelFile, double finalTime=0.5);

int main()
{
	SimTK::Array_<std::string> failures;
/*
	try {// Baseline perfromance without wrapping
		simulateModelWithMuscles("test_nowrap_vasint.osim");}
	catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
		failures.push_back("test_nowrap_vasint"); }

	try{// performance with cylnder wrapping
		simulateModelWithMuscles("test_wrapping_vasint.osim");}
	catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
		failures.push_back("test_wrapping_vasint (cylinder wrap)"); }

	try{// performance with ellipsoid wrapping
		simulateModelWithMuscles("test_wrapEllipsoid_vasint.osim");}
	catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
		failures.push_back("test_wrapEllipsoid_vasint"); }

	try{// performance with multiple muscles and no wrapping
		simulateModelWithMuscles("gait2392_pelvisFixed.osim", 0.1);}
	catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
		failures.push_back("gait2392_pelvisFixed (no wrapping)"); }

	try{// performance with multiple muscles and wrapping
		simulateModelWithMuscles("Arnold2010_pelvisFixed.osim", 0.05);}
	catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
		failures.push_back("Arnold2010_pelvisFixed (multiple wrap)"); }
*/
	try{// performance with multiple muscles and wrapping in upper-exremity
		simulateModelWithMuscles("TestShoulderModel.osim", 0.02);}
	catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
		failures.push_back("TestShoulderModel (multiple wrap)"); }
    
	if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
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
	double control = 0.5;

	// Create a prescribed controller that simply applies a function of the force
	PrescribedController actuatorController;
	actuatorController.setActuators(osimModel.updActuators());
	for (int i=0; i<actuatorController.getActuatorSet().getSize(); i++) {
		actuatorController.prescribeControlForActuator(i, new Constant(control));
	}

	// add the controller to the model
	osimModel.addController(&actuatorController);
	osimModel.disownAllComponents(); // because PrescribedController is on stack

	// Initialize the system and get the state representing the state system
	SimTK::State& si = osimModel.initSystem();

	const Set<Muscle>& muscles = osimModel.getMuscles();
	for (int i=0; i<muscles.getSize(); i++){
		muscles[i].setActivation(si, control); //setDisabled(si, true);
	}
	osimModel.equilibrateMuscles(si); 

	osimModel.printBasicInfo(cout);

    // Dump model back out; no automated test provided here though.
    // osimModel.print(osimModel.getName() + "_out.osim");

	// Create the integrator and manager for the simulation.
	const double accuracy = 1.0e-4;
	SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
	integrator.setAccuracy(accuracy);

	Manager manager(osimModel, integrator);

	// Integrate from initial time to final time
	manager.setInitialTime(initialTime);
	manager.setFinalTime(finalTime);
	cout << "\nIntegrating from " << initialTime << " to " << finalTime << endl;
	
	const double start = SimTK::realTime();
	manager.integrate(si);
	cout << "simulation time = " << SimTK::realTime()-start 
         << " seconds (wallclock time)\n" << endl;

	// Save the simulation results
	Storage states(manager.getStateStorage());
	states.print(osimModel.getName()+"_states.sto");
	osimModel.updSimbodyEngine().convertRadiansToDegrees(states);
	states.setWriteSIMMHeader(true);
	states.print(osimModel.getName()+"_states_degrees.mot");

}// end of simulateModelWithSingleMuscle()