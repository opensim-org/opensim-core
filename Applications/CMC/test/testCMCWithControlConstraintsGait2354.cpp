/* -------------------------------------------------------------------------- *
 *           OpenSim:  testCMCWithControlConstraintsGait2354.cpp              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

void testGait2354();

int main() {
    Object::renameType("Thelen2003Muscle", "Thelen2003Muscle_Deprecated");
    //Object::renameType("Thelen2003Muscle", "Millard2012AccelerationMuscle");
    //Object::renameType("Thelen2003Muscle", "Millard2012EquilibriumMuscle");
    SimTK::Array_<std::string> failures;

    try {testGait2354();}
    catch (const std::exception& e)
        {  cout << e.what() <<endl; failures.push_back("testGait2354"); }

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done" << endl;

    return 0;
}

void testGait2354() {
    cout<<"\n******************************************************************" << endl;
    cout << "*                          testGait2354                          *" << endl;
    cout << "******************************************************************\n" << endl;

    CMCTool cmc("subject01_Setup_CMC.xml");
    cmc.run();

    Storage results("subject01_ResultsCMC/subject01_walk1_Kinematics_q.sto");
    Storage standard("subject01_walk1_RRA_Kinematics_initial_q.sto");

    int nq = results.getColumnLabels().getSize()-1;

    // Tracking kinematics angles in degrees should be within 2 degrees
    Array<double> rms_tols(2.0, nq);
    rms_tols[3] = 0.005; // pelvis translations in m should be with 5mm
    rms_tols[4] = 0.005;
    rms_tols[5] = 0.005;

    CHECK_STORAGE_AGAINST_STANDARD(results, standard, rms_tols, __FILE__, __LINE__, "testGait2354 tracking failed");

    Storage results2("subject01_ResultsCMC/subject01_walk1_states.sto");
    Storage standard2("std_subject01_walk1_states.sto");

    Array<string> col_labels = standard2.getColumnLabels();
    Array<double> rms_tols2(0.1, col_labels.getSize()-1);
    for (int i = 23; i < 46; ++i){
        rms_tols2[i] = 0.75; // velocities
    }
    for (int i = 46; i < rms_tols2.getSize(); ++i){
        rms_tols2[i] = 0.15; // muscle activations and fiber-lengths
    }

    CHECK_STORAGE_AGAINST_STANDARD(results2, standard2, rms_tols2, __FILE__, __LINE__, "testGait2354 states failed");

    cout << "\n testGait2354 passed\n" << endl;
}
