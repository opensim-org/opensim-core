/* -------------------------------------------------------------------------- *
 *           OpenSim:  testCMCWithControlConstraintsRunningModel.cpp          *
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

void testRunningModel();

int main() {

    Object::renameType("Thelen2003Muscle", "Thelen2003Muscle_Deprecated");
    //Object::renameType("Thelen2003Muscle", "Millard2012AccelerationMuscle");
    //Object::renameType("Thelen2003Muscle", "Millard2012EquilibriumMuscle");
    SimTK::Array_<std::string> failures;

    try {testRunningModel();}
    catch (const std::exception& e)
        {  cout << e.what() <<endl; failures.push_back("testRunningModel"); }

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done" << endl;

    return 0;
}

void testRunningModel()
{
    cout<<"\n******************************************************************" << endl;
    cout << "*                     testRunningModel                     *" << endl;
    cout << "******************************************************************\n" << endl;
    CMCTool cmc("runningModel_Setup_CMC_test.xml");
    cmc.run();

    Storage results("runningModel_CMC_Results/runningModel_CMC_test_Kinematics_q.sto");
    Storage standard("runningModel_Kinematics_q.sto");

    int nq = results.getColumnLabels().getSize()-1;

    // Tracking kinematics angles in degrees should be within 3 degrees
    std::vector<double> rms_tols(nq, 3.00);
    rms_tols[3] = 0.0025; // pelvis translations in m should be with 2.5mm
    rms_tols[4] = 0.0025;
    rms_tols[5] = 0.0025;

    CHECK_STORAGE_AGAINST_STANDARD(results, standard, rms_tols, __FILE__, __LINE__,
        "testRunningModel tracking failed");

    Storage results_states("runningModel_CMC_Results/runningModel_CMC_test_states.sto");
    Storage standard_states("std_runningModel_CMC_states.sto");

    int nc = results_states.getColumnLabels().getSize()-1;

    // already passed tracking kinematics so focus on muscle states
    std::vector<double> rms_states_tols(nc, 0.6);
    for(int i = nq; i< 2*nq; ++i)
    {
        rms_states_tols[i] = 0.2; // velocities
    }

    CHECK_STORAGE_AGAINST_STANDARD(results_states, standard_states, rms_states_tols,
        __FILE__, __LINE__, "testRunningModel activations failed");

    cout << "\n testRunningModel passed\n" << endl;
}
