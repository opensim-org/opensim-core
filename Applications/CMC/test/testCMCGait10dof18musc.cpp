/* -------------------------------------------------------------------------- *
 *                           OpenSim:  testCMCGait10dof18musc.cpp             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
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

    // redo with the Millard2012EquilibriumMuscle 
    Object::renameType("Thelen2003Muscle", "Millard2012EquilibriumMuscle");

    try {testGait10dof18musc();}
    catch (const std::exception& e)
        {  cout << e.what() <<endl; failures.push_back("testGait10dof18musc_Millard"); }

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done" << endl;

    return 0;
}

void testGait10dof18musc() {
    cout<<"\n******************************************************************" << endl;
    cout << "*                      testGait10dof18musc                       *" << endl;
    cout << "******************************************************************\n" << endl;
    CMCTool cmc("gait10dof18musc_Setup_CMC.xml");
    cmc.run();

    Storage results("gait10dof18musc_ResultsCMC/walk_subject_states.sto");
    Storage temp("gait10dof18musc_std_walk_subject_states.sto");

    Storage *standard = new Storage();
    cmc.getModel().formStateStorage(temp, *standard);

    Array<double> rms_tols(0, 2*10+2*18);
    rms_tols[0]  = 0.00004;  // ground_pelvis/pelvis_tilt/value
    rms_tols[1]  = 0.001;    // ground_pelvis/pelvis_tilt/speed
    rms_tols[2]  = 0.000002; // ground_pelvis/pelvis_tx/value
    rms_tols[3]  = 0.00002;  // ground_pelvis/pelvis_tx/speed
    rms_tols[4]  = 0.000004; // ground_pelvis/pelvis_ty/value
    rms_tols[5]  = 0.0001;   // ground_pelvis/pelvis_ty/speed
    rms_tols[6]  = 0.00006;  // hip_r/hip_flexion_r/value
    rms_tols[7]  = 0.002;    // hip_r/hip_flexion_r/speed
    rms_tols[8]  = 0.00006;  // hip_l/hip_flexion_l/value
    rms_tols[9]  = 0.002;    // hip_l/hip_flexion_l/speed
    rms_tols[10] = 0.00004;  // back/lumbar_extension/value
    rms_tols[11] = 0.001;    // back/lumbar_extension/speed
    rms_tols[12] = 0.00005;  // knee_r/knee_angle_r/value
    rms_tols[13] = 0.001;    // knee_r/knee_angle_r/speed
    rms_tols[14] = 0.00008;  // knee_l/knee_angle_l/value
    rms_tols[15] = 0.0007;   // knee_l/knee_angle_l/speed
    rms_tols[16] = 0.00003;  // ankle_r/ankle_angle_r/value
    rms_tols[17] = 0.003;    // ankle_r/ankle_angle_r/speed
    rms_tols[18] = 0.0001;   // ankle_l/ankle_angle_l/value
    rms_tols[19] = 0.003;    // ankle_l/ankle_angle_l/speed
    rms_tols[20] = 0.008;    // hamstrings_r/activation
    rms_tols[21] = 0.00005;  // hamstrings_r/fiber_length
    rms_tols[22] = 0.0007;   // bifemsh_r/activation
    rms_tols[23] = 0.00001;  // bifemsh_r/fiber_length
    rms_tols[24] = 0.004;    // glut_max_r/activation
    rms_tols[25] = 0.00002;  // glut_max_r/fiber_length
    rms_tols[26] = 0.0007;   // iliopsoas_r/activation
    rms_tols[27] = 0.00001;  // iliopsoas_r/fiber_length
    rms_tols[28] = 0.00001;  // rect_fem_r/activation
    rms_tols[29] = 0.000001; // rect_fem_r/fiber_length
    rms_tols[30] = 0.002;    // vasti_r/activation
    rms_tols[31] = 0.00001;  // vasti_r/fiber_length
    rms_tols[32] = 0.005;    // gastroc_r/activation
    rms_tols[33] = 0.00001;  // gastroc_r/fiber_length
    rms_tols[34] = 0.0009;   // soleus_r/activation
    rms_tols[35] = 0.00001;  // soleus_r/fiber_length
    rms_tols[36] = 0.0002;   // tib_ant_r/activation
    rms_tols[37] = 0.00001;  // tib_ant_r/fiber_length
    rms_tols[38] = 0.0008;   // hamstrings_l/activation
    rms_tols[39] = 0.00001;  // hamstrings_l/fiber_length
    rms_tols[40] = 0.002;    // bifemsh_l/activation
    rms_tols[41] = 0.00001;  // bifemsh_l/fiber_length
    rms_tols[42] = 0.0002;   // glut_max_l/activation
    rms_tols[43] = 0.00001;  // glut_max_l/fiber_length
    rms_tols[44] = 0.0007;   // iliopsoas_l/activation
    rms_tols[45] = 0.00001;  // iliopsoas_l/fiber_length
    rms_tols[46] = 0.0004;   // rect_fem_l/activation
    rms_tols[47] = 0.00001;  // rect_fem_l/fiber_length
    rms_tols[48] = 0.0004;   // vasti_l/activation
    rms_tols[49] = 0.00001;  // vasti_l/fiber_length
    rms_tols[50] = 0.002;    // gastroc_l/activation
    rms_tols[51] = 0.00001;  // gastroc_l/fiber_length
    rms_tols[52] = 0.002;    // soleus_l/activation
    rms_tols[53] = 0.00003;  // soleus_l/fiber_length
    rms_tols[54] = 0.004;    // tib_ant_l/activation
    rms_tols[55] = 0.00004;  // tib_ant_l/fiber_length

    CHECK_STORAGE_AGAINST_STANDARD(results, *standard, rms_tols, __FILE__, __LINE__, "testArm26 failed");

    const string& muscleType = cmc.getModel().getMuscles()[0].getConcreteClassName();
    cout << "\ntestGait10dof18musc "+muscleType+" passed\n" << endl;
}
