/* -------------------------------------------------------------------------- *
 *                    OpenSim:  testCMCEMGDrivenArm_Thelen.cpp                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Tools/CMCTool.h>
#include <OpenSim/Tools/ForwardTool.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <opensim-core/tests/opensim_tests_config.h>

#include <catch2/catch_all.hpp>

#include <filesystem>

using namespace OpenSim;
using namespace std;

TEST_CASE("testCMCEMGDrivenArm_Thelen") {
    cout<<"\n******************************************************************" << endl;
    cout << "*               testCMCEMGDrivenArm_Thelen                       *" << endl;
    cout << "******************************************************************\n" << endl;
    const auto arm26Dir = opensim_tests_resources_directory() / "models" / "Arm26";
    // Absolute results dir under the CWD (tool writes results relative to the
    // Setup XML's now read-only resources directory).
    const string cmcResultsDir =
            (std::filesystem::current_path() / "Results_Arm26_EMG_Thelen").string();
    CMCTool cmc((arm26Dir / "arm26_Setup_ComputedMuscleControl_EMG.xml").string());
    cmc.setResultsDir(cmcResultsDir);
    cmc.run();

    Storage results(cmcResultsDir + "/arm26_states.sto"),
            temp("std_arm26_states.sto");
    Storage *standard = new Storage();
    cmc.getModel().formStateStorage(temp, *standard);

    std::vector<double> rms_tols(2*2+2*6, 0.01);
    rms_tols[1] = 0.05;  // shoulder speed
    rms_tols[3] = 0.05;  // elbow speed
    rms_tols[4] = 0.10;  // trilong
    rms_tols[6] = 0.25;  // trilat normally off but because of bicep long EMG tracking it turns on
    rms_tols[8] = 0.25;  // trimed normally off but because of bicep long EMG tracking it turns on
    rms_tols[10] = 0.50;  // biceps long normally low but because of EMG tracking should be on more
    rms_tols[12] = 0.50;  // biceps short normally on but because of EMG tracking should be lower

    CHECK_STORAGE_AGAINST_STANDARD(results, *standard, rms_tols,
                                    __FILE__, __LINE__,
                                    "testCMCEMGDrivenArm_Thelen failed");

    const string& muscleType = cmc.getModel().getMuscles()[0].getConcreteClassName();
    cout << "\ntestCMCEMGDrivenArm_"+muscleType+ " passed\n" << endl;
}

