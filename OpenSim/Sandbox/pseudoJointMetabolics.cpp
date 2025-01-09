/* -------------------------------------------------------------------------- *
 *                    OpenSim:  pseudoJointMetabolics.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Thomas Uchida                                                   *
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
Hypothetical code for computing the metabolic power attributable to each lower
extremity joint given a model and a states file. Metabolic power is allotted
based on relative moments (equivalently, moment arms) in the sagittal plane.
*/

#include <OpenSim/OpenSim.h>

int main()
{
    try {
        // Load model and states file.
        Model model("subject01.osim");
        StatesTrajectory states("CMC_results_states.sto");
        
        // Append metabolics model. This xml file should contain slow-twitch
        // fiber ratios, muscle masses, and other parameters required by the
        // Umberger2010 model.
        metabolics Umberger2010MuscleMetabolics("gait2392_metabolics.xml");
        model.append(metabolics);
        
        // Create and configure reporter. Collect metabolic power and moment
        // arms for each muscle; allocate power based on relative moment arms.
        Reporter reporter("subject01_metabolics_analysis.csv");
        ReporterChannel& metabolicPower =
            reporter.addChannel(metabolics.getOutput("total_rate"));
        ReporterChannel& momentArms =
            reporter.addChannel(model.getMuscles().getOutput("moment_arm", "all"));
        
        for (int i=0; i<model.getMuscles().size(); ++i) {
            Muscle& currMuscle = model.getMuscle(i);
            ReporterChannel& currMusclePower =
                metabolicPower.get(currMuscle.getName());
            ReporterChannel& currMuscleMomentArms =
                momentArms.get(currMuscle.getName());
            
            // Calculate the absolute value of each moment arm.
            ReporterChannel& momentArmHipR = reporter.addCalculation("abs",
                currMuscleMomentArms.get("hip_flexion_r") );
            ReporterChannel& momentArmKneeR = reporter.addCalculation("abs",
                currMuscleMomentArms.get("knee_flexion_r") );
            ReporterChannel& momentArmAnkleR = reporter.addCalculation("abs",
                currMuscleMomentArms.get("ankle_flexion_r") );
            ReporterChannel& momentArmHipL = reporter.addCalculation("abs",
                currMuscleMomentArms.get("hip_flexion_l") );
            ReporterChannel& momentArmKneeL = reporter.addCalculation("abs",
                currMuscleMomentArms.get("knee_flexion_l") );
            ReporterChannel& momentArmAnkleL = reporter.addCalculation("abs",
                currMuscleMomentArms.get("ankle_flexion_l") );
            
            // Calculate the sum of all moment arms.
            ReporterChannel& momentArmTotal = reporter.addCalculation("+",
                momentArmHipR, momentArmKneeR, momentArmAnkleR,
                momentArmHipL, momentArmKneeL, momentArmAnkleL);
            
            // Do not report these internal calculations.
            momentArmHipR.mute();
            momentArmKneeR.mute();
            momentArmAnkleR.mute();
            momentArmHipL.mute();
            momentArmKneeL.mute();
            momentArmAnkleL.mute();
            momentArmTotal.mute();
            
            // Assign appropriate fraction of metabolic power to each joint.
            reporter.addCalculation("*", currMusclePower,
                reporter.addCalculation("/", momentArmHipR, momentArmTotal) );
            reporter.addCalculation("*", currMusclePower,
                reporter.addCalculation("/", momentArmKneeR, momentArmTotal) );
            reporter.addCalculation("*", currMusclePower,
                reporter.addCalculation("/", momentArmAnkleR, momentArmTotal) );
            reporter.addCalculation("*", currMusclePower,
                reporter.addCalculation("/", momentArmHipL, momentArmTotal) );
            reporter.addCalculation("*", currMusclePower,
                reporter.addCalculation("/", momentArmKneeL, momentArmTotal) );
            reporter.addCalculation("*", currMusclePower,
                reporter.addCalculation("/", momentArmAnkleL, momentArmTotal) );
        }
        
        // Create, configure, and execute study.
        study = InverseStudy(model, states);
        study.append(reporter);
        study.run();
        
    }
    catch (const std::exception& ex)
    {
        std::cout << "Exception: " << ex.what() << std:: endl;
        return 1;
    }
    catch (...)
    {
        std::cout << "Unrecognized exception" << std::endl;
        return 1;
    }
    
    return 0;
}
