/* ------------------------------------------------------------------------- *
*             OpenSim:  pseudoMuscleLinesOfAction.cpp                        *
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


/*
Demonstrates how users can extract muscles line of action during a motion. The
motion is specified by a sto, mot or c3d file
*/

int main()
{
    try {
      
        Model model("gaitmodel.osim");
        // Trajectory could be replaced by a specific class if already exists
        Trajectory traj("trial.c3d");
        // Reporter needs only to know about the model and which muscles to report
        // Could also have events e.g. between heel_strike and toe_off
        MuscleLineOfActionReporter musclePathReporter(model);
        // Either filtering by Group, or wildcards would be useful
        // another option is to pass in a filter method that selects what objects to report
        musclePathReporter.selectMuscles("R_hip_flex");

        for (auto timeFrame : traj) {
            // Don't necessarily have to pass state or state-reference around
            // but knowing that the model is stateless this maybe necessary
            State s = model.applyFrame(timeFrame);
            // Results are accumulated into internal table(s) 
            musclePathReporter.computePaths(s);
            
        }
        // I expect for every frame to get a sequence of [body-point, vector] for each selected muscle
        // this could be variable length depending on wrapping/via-point(s) engagement
        musclePathReporter.print("musclePaths.sto");
    }
    catch (const std::exception& ex)
    {
        std::cout << "Exception: " << ex.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cout << "Unrecognized exception " << std::endl;
        return 1;
    }
    
    return 0;
}
