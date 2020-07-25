/* -------------------------------------------------------------------------- *
 *                     OpenSim:  OptimizationExample.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Samuel R. Hamner, Ajay Seth                                     *
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

/* 
 *  Example of an OpenSim program that optimizes the performance of a model.
 *  The main() loads the arm26 model and maximizes the forward velocity of
 *  the hand during a muscle-driven forward simulation by finding the set
 *  of (constant) controls.
 */

//==============================================================================
//==============================================================================
#include <OpenSim/OpenSim.h>
#include "OpenSim/Common/STOFileAdapter.h"
#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC

using namespace OpenSim;
using namespace SimTK;
using namespace std;

// Global variables to define integration time window, optimizer step count,
// the best solution.
int stepCount = 0; 
const double initialTime = 0.0;
const double finalTime = 0.25;
const double desired_accuracy = 1.0e-5;
double bestSoFar = Infinity;

class ExampleOptimizationSystem : public OptimizerSystem {
public:
    /* Constructor class. Parameters passed are accessed in the objectiveFunc() class. */
    ExampleOptimizationSystem(int numParameters, State& s, Model& aModel): 
        OptimizerSystem(numParameters), 
        si(s),
        osimModel(aModel)
    {}
                
    int objectiveFunc(const Vector &newControls,
        bool new_coefficients, Real& f) const override {

        // make a copy of the initial states
        State s = si;

        // Update the control values
        osimModel.updDefaultControls() = newControls;
                
        // Integrate from initial time to final time
        Manager manager(osimModel);
        manager.setIntegratorAccuracy(desired_accuracy);
        s.setTime(initialTime);

        osimModel.getMultibodySystem().realize(s, Stage::Acceleration);

        manager.initialize(s);
        s = manager.integrate(finalTime);

        /* Calculate the scalar quantity we want to minimize or maximize. 
        *  In this case, we're maximizing forward velocity of the
        *  forearm/hand mass center, so to maximize, compute velocity 
        *  and multiply it by -1.
        */
        const auto& hand = osimModel.getBodySet().get("r_ulna_radius_hand");
        osimModel.getMultibodySystem().realize(s, Stage::Velocity);
        Vec3 massCenter = hand.getMassCenter();
        Vec3 velocity = hand.findStationVelocityInGround(s, massCenter);
        f = -velocity[0];
        stepCount++;
        
        // Use an if statement to only store and print the results of an 
        //  optimization step if it is better than a previous result.
        if(f < bestSoFar) {
            bestSoFar = f;
            cout << "\nobjective evaluation #: " << stepCount << "  controls = "
                 << newControls <<  " bestSoFar = " << f << std::endl;
        }

      return 0;

   }    

private:
    State& si;
    Model& osimModel;
    SimTK::ReferencePtr<RungeKuttaMersonIntegrator> p_integrator;

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

        // Use Millard2012Equilibrium muscles with rigid tendons for better
        // performance.
        Object::renameType("Thelen2003Muscle", "Millard2012EquilibriumMuscle");
    
        // Create a new OpenSim model. This model is similar to the arm26 model,
        // but without wrapping surfaces for better performance.
        Model osimModel("Arm26_Optimize.osim");
        
        // Initialize the system and get the state representing the state system
        State& si = osimModel.initSystem();

        // Initialize the starting shoulder angle.
        const CoordinateSet& coords = osimModel.getCoordinateSet();
        coords.get("r_shoulder_elev").setValue(si, -1.57079633);

        // Set the initial muscle activations and make all tendons rigid.
        const Set<Muscle> &muscleSet = osimModel.getMuscles();
        for(int i=0; i<muscleSet.getSize(); ++i) {
            muscleSet[i].setActivation(si, 0.01);
            muscleSet[i].setIgnoreTendonCompliance(si, true);
        }
    
        // Make sure the muscles states are in equilibrium
        osimModel.equilibrateMuscles(si);

        // The number of controls will equal the number of muscles in the model!
        int numControls = osimModel.getNumControls();
        
        // Initialize the optimizer system we've defined.
        ExampleOptimizationSystem sys(numControls, si, osimModel);
        Real f = NaN;
        
        /* Define initial values and bounds for the controls to optimize */
        Vector controls(numControls, 0.02);
        controls[3] = 0.99;
        controls[4] = 0.99;
        controls[5] = 0.99;

        Vector lower_bounds(numControls, 0.01);
        Vector upper_bounds(numControls, 0.99);

        sys.setParameterLimits( lower_bounds, upper_bounds );
        
        // Create an optimizer. Pass in our OptimizerSystem
        // and the name of the optimization algorithm.
        Optimizer opt(sys, SimTK::LBFGSB);
        //Optimizer opt(sys, InteriorPoint);

        // Specify settings for the optimizer
        opt.setConvergenceTolerance(0.1);
        opt.useNumericalGradient(true, desired_accuracy);
        opt.setMaxIterations(2);
        opt.setLimitedMemoryHistory(500);

        // Optimize it!
        f = opt.optimize(controls);
            
        cout << "Elapsed time = " << (std::clock()-startTime)/CLOCKS_PER_SEC << "s" << endl;
        
        const Set<Actuator>& actuators = osimModel.getActuators();
        for(int i=0; i<actuators.getSize(); ++i){
            cout << actuators[i].getName() << " control value = " << controls[i] << endl;
        }

        cout << "\nMaximum hand velocity = " << -f << "m/s" << endl;

        cout << "OpenSim example completed successfully." << endl;
        
        // Dump out optimization results to a text file for testing
        ofstream ofile; 
        ofile.open("Arm26_optimization_result"); 
        for(int i=0; i<actuators.getSize(); ++i)
            ofile << controls[i] << endl;
        ofile << -f <<endl;
        ofile.close(); 

        // Re-run simulation with optimal controls.
        Manager manager(osimModel);
        manager.setIntegratorAccuracy(desired_accuracy);
        osimModel.updDefaultControls() = controls;

        // Integrate from initial time to final time.
        si.setTime(initialTime);
        osimModel.getMultibodySystem().realize(si, Stage::Acceleration);
        manager.initialize(si);
        si = manager.integrate(finalTime);

        auto statesTable = manager.getStatesTable();
        STOFileAdapter_<double>::write(statesTable, 
                                      "Arm26_optimized_states.sto");
    }
    catch (const std::exception& ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
    
    // End of main() routine.
    return 0;
}
