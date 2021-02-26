/* -------------------------------------------------------------------------- *
 *                           OpenSim:  testCMCLeg6Dof9Musc.cpp                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
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

void testLeg6Dof9MuscSwing();
void testLeg6Dof9MuscStance();

int main() {

    SimTK::Array_<std::string> failures;

    try {
        testLeg6Dof9MuscSwing();
    } catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testLeg6Dof9MuscSwing");
    }
    try {
        testLeg6Dof9MuscStance();
    } catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testLeg6Dof9MuscStance");
    }

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done" << endl;

    return 0;
}

// Perform regression test with standard generated from Leg6Dof9Musc
// example in OpenSim 4.1
void testLeg6Dof9MuscSwing() {
    cout << "\n****************************************************************"
            "**"
         << endl;
    cout << "*                      testLeg6Dof9Musc Swing                     "
            "*"
         << endl;
    cout << "******************************************************************"
            "\n"
         << endl;
    CMCTool cmc("leg6dof9musc_setup_Swing.xml");
    CoordinateSet& coordSet = cmc.getModel().updCoordinateSet();
    // Lock pelvis coordinates per tutorial
    for (int i = 0; i < 3; i++) coordSet[i].setDefaultLocked(true);
    if (!cmc.run())
        OPENSIM_THROW(Exception, "testLeg6Dof9Musc Swing failed to complete.");

    Storage results("Leg6Dof9MuscSwing/leg6dof9musc_states.sto");
    Storage temp("Leg6Dof9MuscSwing_std_leg6dof9musc_states.sto");

    Storage* standard = new Storage();
    cmc.getModel().formStateStorage(temp, *standard);

    int nstates = standard->getColumnLabels().size() - 1;

    // angles and speeds within 0.05 rads and 0.05 rad/s;
    // and activations to within 5%
    std::vector<double> rms_tols(nstates, 0.05);

    CHECK_STORAGE_AGAINST_STANDARD(results, *standard, rms_tols, __FILE__,
            __LINE__, "testLeg6Dof9Musc Swing failed");

    cout << "\ntestLeg6Dof9Musc Swing passed\n" << endl;
}
void testLeg6Dof9MuscStance() {
    cout << "\n****************************************************************"
            "**"
         << endl;
    cout << "*                      testLeg6Dof9Musc Stance                     "
            "*"
         << endl;
    cout << "******************************************************************"
            "\n"
         << endl;
    CMCTool cmc("leg69_Setup_CMC_Stance.xml");

    if (!cmc.run())
        OPENSIM_THROW(Exception, "testLeg6Dof9Musc Stance failed to complete.");

    Storage results("Leg6Dof9MuscStance/leg6dof9musc_RRA_adjusted_states.sto");
    Storage temp("leg6dof9musc_std_RRA_adjusted_states.sto");

    Storage* standard = new Storage();
    cmc.getModel().formStateStorage(temp, *standard);

    int nstates = standard->getColumnLabels().size() - 1;

    // angles and speeds within 0.1 rads and 0.1 rad/s;
    // and activations to within 10%
    std::vector<double> rms_tols(nstates, 0.1);

    CHECK_STORAGE_AGAINST_STANDARD(results, *standard, rms_tols, __FILE__,
            __LINE__, "testLeg6Dof9Musc Stance failed");

    cout << "\ntestLeg6Dof9Musc Stance passed\n" << endl;
}



