/* -------------------------------------------------------------------------- *
 *                     OpenSim:  OptimizationExample.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Samuel R. Hamner                                                *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/* 
 *  Below is an example of an OpenSim application that provides its own 
 *  main() routine.  This application is a forward simulation of tug-of-war between two
 *  muscles pulling on a block.
 */

// Author:  Samuel Hamner

//==============================================================================
//==============================================================================
#include <OpenSim/OpenSim.h>
#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC

using namespace OpenSim;
using namespace SimTK;
// Global variables to define integration time window, optimizer step count,
// the best solution.
int stepCount = 0;
double initialTime = 0.0;
double finalTime = 0.25;
double bestSoFar = Infinity;

class ExampleOptimizationSystem : public OptimizerSystem {
   public:

	   /* Constructor class. Parameters passed are accessed in the objectiveFunc() class. */
	   ExampleOptimizationSystem(int numParameters, State& s, Model& aModel): 
             numControls(numParameters), OptimizerSystem(numParameters), si(s), osimModel(aModel){}
			 	
	int objectiveFunc(  const Vector &newControls, const bool new_coefficients, Real& f ) const {

        // make a copy of out initial states
        State s = si;

        // Access the controller set of the model and update the control values
		((ControlSetController *)(&osimModel.updControllerSet()[0]))->updControlSet()->setControlValues(initialTime, &newControls[0]);
		((ControlSetController *)(&osimModel.updControllerSet()[0]))->updControlSet()->setControlValues(finalTime, &newControls[0]);
        
		// Create the integrator for the simulation.
		RungeKuttaMersonIntegrator integrator(osimModel.getSystem());
		integrator.setAccuracy(1.0e-5);

		// Create a manager to run the simulation
		Manager manager(osimModel, integrator);

		// Integrate from initial time to final time
		manager.setInitialTime(initialTime);
		manager.setFinalTime(finalTime);
		osimModel.getSystem().realize(s, Stage::Acceleration);
		manager.integrate(s);

		/* Calculate the scalar quantity we want to minimize or maximize. 
		*  In this case, we’re maximizing forward velocity of the 
		*  forearm/hand mass center, so to maximize, compute velocity 
		*  and multiply it by -1.
		*/
		Vec3 massCenter;
		Vec3 velocity;
		osimModel.getBodySet().get("r_ulna_radius_hand").getMassCenter(massCenter);
		osimModel.getSystem().realize(s, Stage::Acceleration);
		osimModel.getSimbodyEngine().getVelocity(s, osimModel.getBodySet().get("r_ulna_radius_hand"), massCenter, velocity);
		
		f = -velocity[0];
		stepCount++;
		
		// Store and print the results of a "random sample"
		if( stepCount == 23 ){
			Storage statesDegrees(manager.getStateStorage());
			osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
			statesDegrees.print("Arm26_randomSample_states_degrees.sto");
		}
		// Store and print the  results of the first step.
		else if( stepCount == 1){ 
			Storage statesDegrees(manager.getStateStorage());
			osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
			statesDegrees.print("Arm26_noActivation_states_degrees.sto");
		}
		// Use an if statement to only store and print the results of an 
		//  optimization step if it is better than a previous result.
		else if( f < bestSoFar){
			Storage statesDegrees(manager.getStateStorage());
			osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
			statesDegrees.print("Arm26_bestSoFar_states_degrees.sto");
			bestSoFar = f;
			std::cout << "\nOptimization Step #: " << stepCount << "  controls = " << newControls <<  " bestSoFar = " << f << std::endl;
		}		    

      return(0);

   }	

private:
    int numControls;
	State& si;
	Model& osimModel;
 };

//______________________________________________________________________________
/**
 * Define an optimization problem that finds a set of muscle controls to maximize 
 * the forward velocity of the forearm/hand segment mass center. 
 */
int main()
{
	try {
		std::clock_t startTime = std::clock();	
		// Create a new OpenSim model
		LoadOpenSimLibrary("osimActuators");
		Model osimModel("Arm26_Optimize.osim");
		
		// Define the initial and final controls
		ControlLinear *control_TRIlong = new ControlLinear();
		ControlLinear *control_TRIlat = new ControlLinear();
		ControlLinear *control_TRImed = new ControlLinear();
		ControlLinear *control_BIClong = new ControlLinear();
		ControlLinear *control_BICshort = new ControlLinear();
		ControlLinear *control_BRA = new ControlLinear();

		/* NOTE: Each contoller must be set to the corresponding 
		 *		muscle name in the model file. */
		control_TRIlong->setName("TRIlong"); control_TRIlat->setName("TRIlat"); 
		control_TRImed->setName("TRImed"); control_BIClong->setName("BIClong");
		control_BICshort->setName("BICshort"); control_BRA->setName("BRA");
		
		ControlSet *muscleControls = new ControlSet();
		muscleControls->append(control_TRIlong); muscleControls->append(control_TRIlat);
		muscleControls->append(control_TRImed); muscleControls->append(control_BIClong);
		muscleControls->append(control_BICshort); muscleControls->append(control_BRA);
		
		ControlSetController *muscleController = new ControlSetController();
		muscleController->setControlSet(muscleControls);
        muscleController->setName("MuscleController");

		// Add the controller to the model
		osimModel.addController(muscleController);
		
		// Initialize the system and get the state representing the state system
		State& si = osimModel.initSystem();

		// Define the initial muscle states
		const OpenSim::Set<OpenSim::Actuator> &muscleSet = osimModel.getActuators();
     	for(int i=0; i< muscleSet.getSize(); i++ ){
			Actuator* act = &muscleSet.get(i);
			OpenSim::Muscle* mus = dynamic_cast<Muscle*>(act);
			mus->setDefaultActivation(0.5);
			mus->setDefaultFiberLength(0.1);
			//mus->initState(si);
		}
	
		// Make sure the muscles states are in equilibrium
		osimModel.equilibrateMuscles(si);

		// The number of controls will equal the number of muscles in the model!
		int numControls = 6;
		
		// Initialize the optimizer system we've defined.
		ExampleOptimizationSystem sys(numControls, si, osimModel);
		Real f = NaN;
		
		/* Define and set bounds for the parameter we will optimize */
		Vector lower_bounds(numControls);
		Vector upper_bounds(numControls);

		for(int i=0;i<numControls;i++) {
			lower_bounds[i] = 0.01;
			upper_bounds[i] = 1.0;
		}

		sys.setParameterLimits( lower_bounds, upper_bounds );
		
        // set the initial values (guesses) for the controls
		Vector controls(numControls);
        controls[0]  = 0.01;
        controls[1]  = 0.01;
		controls[2]  = 0.01;
		controls[3]  = 0.01;
		controls[4]  = 0.01;
		controls[5]  = 0.01;

		try {
			// Create an optimizer. Pass in our OptimizerSystem
			// and the name of the optimization algorithm.
			 Optimizer opt(sys, SimTK::LBFGSB);
			//Optimizer opt(sys, InteriorPoint);

			// Specify settings for the optimizer
			opt.setConvergenceTolerance(0.2);
			opt.useNumericalGradient(true);
			opt.setMaxIterations(1000);
			opt.setLimitedMemoryHistory(500);
			
			// Optimize it!
			f = opt.optimize(controls);
		}
		catch(const std::exception& e) {
		std::cout << "OptimizationExample.cpp Caught exception :"  << std::endl;
		std::cout << e.what() << std::endl;
		}
		
		// osimModel.print("optimization_model_ARM.osim");
		std::cout << "Elapsed time = " << 1.e3*(std::clock()-startTime)/CLOCKS_PER_SEC << "ms\n";
		char c;
		std::cout << "Press RETURN to end program...\n" << std::endl;
		std::cin.get(c);
	}
    catch (std::exception ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
	
	// End of main() routine.
	return 0;
}
