/* -------------------------------------------------------------------------- *
 *                           OpenSim:  testCMCGait10dof18musc.cpp             *
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

using namespace OpenSim;
using namespace std;

void testGait10dof18musc();

int main() {

    SimTK::Array_<std::string> failures;

    // Model uses Millard2012EquilibriumMuscle type muscles
    try { testGait10dof18musc(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testGait10dof18musc");
    }

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done" << endl;

    return 0;
}

// Perform regression test with standard generated from Gait10dof18musc
// example in OpenSim 3.2
void testGait10dof18musc() {
    cout<<"\n******************************************************************" << endl;
    cout << "*                      testGait10dof18musc                       *" << endl;
    cout << "******************************************************************\n" << endl;
    CMCTool cmc("gait10dof18musc_Setup_CMC.xml");
    const string& muscleType = cmc.getModel().getMuscles()[0].getConcreteClassName();

    if (!cmc.run())
        OPENSIM_THROW(Exception, "testGait10dof18musc " + muscleType +
            " failed to complete.");

    Storage results("gait10dof18musc_ResultsCMC/walk_subject_states.sto");
    Storage temp("gait10dof18musc_std_walk_subject_states.sto");

    Storage *standard = new Storage();
    cmc.getModel().formStateStorage(temp, *standard);

    int nstates = standard->getColumnLabels().size() - 1;

    // angles and speeds within 0.01 rads and 0.01 rad/s;
    // and activations to within 1%
    std::vector<double> rms_tols(nstates, 0.01);

    CHECK_STORAGE_AGAINST_STANDARD(results, *standard, rms_tols,
        __FILE__, __LINE__, "testGait10dof18musc "+ muscleType + " failed");

    cout << "\ntestGait10dof18musc "+ muscleType +" passed\n" << endl;
}


