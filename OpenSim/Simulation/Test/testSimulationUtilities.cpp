/* -------------------------------------------------------------------------- *
 *                   OpenSim:  testSimulationUtilities.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2018 Stanford University and the Authors                *
 * Author(s): Chris Dembia
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

#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimulationUtilities.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>

using namespace OpenSim;
using namespace std;

void testUpdateKinematicsFilesForUpdatedModel();

int main() {
    LoadOpenSimLibrary("osimActuators");

    SimTK_START_TEST("testSimulationUtilities");
        SimTK_SUBTEST(testUpdateKinematicsFilesForUpdatedModel);
    SimTK_END_TEST();
}

void testUpdateKinematicsFilesForUpdatedModel() {
    
    // The model and motion files for this test are from the opensim-models
    // repository. This PR is related to issues #2240 and #2088.
    
    Model model("testSimulationUtilities_leg6dof9musc_20303.osim");
    
    // Ensure the model file has an inconsistent motion type
    // (that it wasn't accidentally updated in the repository).
    SimTK_TEST(!model.getWarningMesssageForMotionTypeInconsistency().empty());
    
    Storage origKinematics("testSimulationUtilities_leg69_IK_stance_pre4.mot");
    auto updatedKinematics = updateKinematicsStorageForUpdatedModel(model,
            origKinematics);
    updatedKinematics->multiplyColumn(
            updatedKinematics->getStateIndex("knee_angle_pat_r"),
            SimTK_RADIAN_TO_DEGREE);
    
    const int numColumns = origKinematics.getColumnLabels().getSize();
    CHECK_STORAGE_AGAINST_STANDARD(*updatedKinematics, origKinematics,
                                   std::vector<double>(numColumns, 1e-14),
                                   __FILE__, __LINE__, "TODO");
}
