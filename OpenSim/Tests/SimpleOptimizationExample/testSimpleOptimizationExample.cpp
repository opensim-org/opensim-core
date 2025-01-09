/* -------------------------------------------------------------------------- *
 *                   OpenSim:  testSimpleOptimizationExample.cpp              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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

// Author: Ayman Habib   

//==============================================================================
//==============================================================================
#include <OpenSim/OpenSim.h>
#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

// step count for troubleshooting
int stepCount = 0;

double bestSoFar = Infinity;

class ExampleOptimizationSystem : public OptimizerSystem {
   public:

       /* Constructor class. Parameters passed are accessed in the objectiveFunc() class. */
       ExampleOptimizationSystem(int numParameters, State& s, Model& aModel): 
             OptimizerSystem(numParameters), 
             si(s),
             osimModel(aModel){}
                
    int objectiveFunc(  const Vector &newControls, bool new_coefficients, Real& f ) const override {

        // make a copy of out initial states
        State s = si;

        // Update the coordinate value of r_elbow_flex
        OpenSim::Coordinate& elbowFlexCoord = osimModel.updCoordinateSet().get("r_elbow_flex");
        elbowFlexCoord.setValue(s, newControls[0]);
        // Now equilibrate muscles at this configuration
        const Set<Muscle> &muscleSet = osimModel.getMuscles();
        // Make sure other muscle states are initialized the same with 1.0 activation, 0.1 fiberLength followed by equilibrium computation
        for(int i=0; i< muscleSet.getSize(); i++ ){
            muscleSet[i].setActivation(s, 1.0);
            const ActivationFiberLengthMuscle* afl = ActivationFiberLengthMuscle::safeDownCast(&muscleSet[i]);
            if (afl) afl->setFiberLength(s, .1);
        }
        // Make sure the muscles states are in equilibrium
        osimModel.equilibrateMuscles(s);

        const OpenSim::Muscle& bicShort = osimModel.getMuscles().get("BICshort");
        // Compute moment arm of BICshort, flip sign since the optimizer tries to minimize rather than maximize
        f = -bicShort.computeMomentArm(s, elbowFlexCoord);

        stepCount++;
        
        if( f < bestSoFar){
            bestSoFar = f;
            cout << "\nobjective evaluation #: " << stepCount << " elbow flexion angle = " << newControls[0]*SimTK_RADIAN_TO_DEGREE <<  " BICshort moment arm  = " << -f << std::endl;
        }           

      return(0);

   }    

private:
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
        // Similar to arm26 model but without wrapping surfaces for better performance
        Model osimModel("Arm26_Optimize.osim");
        
        // Initialize the system and get the state representing the state system
        State& si = osimModel.initSystem();

        // initialize the starting shoulder angle
        const CoordinateSet& coords = osimModel.getCoordinateSet();
        coords.get("r_shoulder_elev").setValue(si, 0.0);

        // Set the initial muscle activations 
        const Set<Muscle> &muscleSet = osimModel.getMuscles();
        for(int i=0; i< muscleSet.getSize(); i++ ){
            muscleSet[i].setActivation(si, 1.0);
            const ActivationFiberLengthMuscle* afl = ActivationFiberLengthMuscle::safeDownCast(&muscleSet[i]);
            afl->setFiberLength(si, .1);
        }
        OpenSim::Coordinate& elbowFlexCoord = osimModel.updCoordinateSet().get("r_elbow_flex");
        elbowFlexCoord.setValue(si, 1.0);
        //osimModel.getMultibodySystem().realize(si, Stage::Velocity);
        // Make sure the muscles states are in equilibrium
        osimModel.equilibrateMuscles(si);
        
        // Initialize the optimizer system we've defined.
        ExampleOptimizationSystem sys(1, si, osimModel);
        Real f = NaN;
        
        /* Define initial values and bounds for the controls to optimize */

        Vector controls(1, 1.0); // 1 radian for default value
        Vector lower_bounds(1, elbowFlexCoord.getRangeMin());
        Vector upper_bounds(1, elbowFlexCoord.getRangeMax());

        sys.setParameterLimits( lower_bounds, upper_bounds );
        
        // Create an optimizer. Pass in our OptimizerSystem
        // and the name of the optimization algorithm.
        Optimizer opt(sys, SimTK::LBFGSB);

        // Specify settings for the optimizer
        opt.setConvergenceTolerance(0.000001);
        opt.useNumericalGradient(true);
        opt.setMaxIterations(1000);
        opt.setLimitedMemoryHistory(500);
            
        // Optimize it!
        f = opt.optimize(controls);
            
        cout << "Elapsed time = " << (std::clock()-startTime)/CLOCKS_PER_SEC << "s" << endl;
        
        ASSERT_EQUAL(f, -0.049390, 1e-5);
        cout << "OpenSim example completed successfully.\n";
    }
    catch (const std::exception& ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
    
    // End of main() routine.
    return 0;
}
