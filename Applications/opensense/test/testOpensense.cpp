/* -------------------------------------------------------------------------- *
 *                            OpenSim:  testOpensense.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Ajay Seth                                          *
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


// INCLUDES
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/OpenSense/OpenSenseUtilities.h>
#include <OpenSim/Simulation/OpenSense/InverseKinematicsStudy.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;


int main()
{

    // Calibrate model and compare result to standard
    Model model = OpenSenseUtilities::calibrateModelFromOrientations(
        "subject07.osim",
        "imuOrientations.sto",
        "pelvis_imu", "z",
        false
        );
    // Previous line produces a model with same name but "calibrated_" prefix.
    Model stdModel{ "std_calibrated_subject07.osim" };
    model.print("calibrated_subject07.osim");
    ASSERT(model == stdModel);

    // Calibrate model from two different standing trials facing
    // opposite directions to verify that heading correction is working
    Model facingX = OpenSenseUtilities::calibrateModelFromOrientations(
        "subject07.osim",
        "MT_012005D6_009-quaternions_calibration_trial_Facing_X.sto",
        "pelvis_imu", "z",
        false);
    facingX.setName("calibrated_FacingX");
    facingX.finalizeFromProperties();

    InverseKinematicsStudy ik_hjc("setup_track_HJC_trial.xml");
    ik_hjc.setModel(facingX);
    ik_hjc.set_results_directory("ik_hjc_" + facingX.getName());
    ik_hjc.run(false);

    // Now facing the opposite direction (negative X)
    Model facingNegX = OpenSenseUtilities::calibrateModelFromOrientations(
        "subject07.osim",
        "MT_012005D6_009-quaternions_calibration_trial_Facing_negX.sto",
        "pelvis_imu", "z",
        false);
    facingNegX.setName("calibrated_FacingNegX");
    facingNegX.finalizeFromProperties();

    ik_hjc.setModel(facingNegX);
    ik_hjc.set_results_directory("ik_hjc_" + facingNegX.getName());
    ik_hjc.run(false);

    Storage ik_X("ik_hjc_" + facingX.getName() + 
        "/ik_MT_012005D6_009-quaternions_RHJCSwinger.mot");

    Storage ik_negX("ik_hjc_" + facingNegX.getName() +
        "/ik_MT_012005D6_009-quaternions_RHJCSwinger.mot");

    int nc = ik_X.getColumnLabels().size();

    // calibration should only result in errors due to smallish (<10degs) 
    // differences in static pose an should be unaffected by the large
    // (90+ degs) change in heading
    CHECK_STORAGE_AGAINST_STANDARD(ik_X, ik_negX,
        std::vector<double>(nc, 10.0), __FILE__, __LINE__,
        "testOpenSense::IK solutions differed due to heading.");

    std::cout << "Done. All testOpensense cases passed." << endl;
    return 0;
}
