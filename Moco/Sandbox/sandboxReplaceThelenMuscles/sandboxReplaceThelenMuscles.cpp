/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxReplaceThelenMuscles.cpp                              *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Prasanna Sritharan                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <Moco/osimMoco.h>
#include <iostream>

#include <OpenSim/OpenSim.h>

using namespace OpenSim;

int main() {

    // invoke a MocoTrack object
    MocoTrack moco;
    moco.setName("mymoco");

    // create a model processor and swap Thelen2003 muscles for
    // DeGrooteFregly2016, add GRFs and reserve actuators
    ModelProcessor mp =
            ModelProcessor("FAIDC1_Residuals_Gmax.osim") |
            ModOpAddExternalLoads("FAIDC1_INITIAL_WALK01_ExternalLoads_NoKinematics.xml") |
            ModOpReplaceMusclesWithDeGrooteFregly2016() |
            ModOpIgnorePassiveFiberForcesDGF() |
            ModOpScaleActiveFiberForceCurveWidthDGF(1.5) |
            ModOpAddReserves(250);

	// print the model to file (for records)
    Model modl = mp.process();
    modl.initializeState();
	modl.print("FAIDC1_Residuals_Gmax_DGFMuscles.osim");

    // set the model
    moco.setModel(mp);

    // set markers data
    moco.setMarkersReferenceFromTRC("FAIDC1_INITIAL_WALK01.trc");
    moco.set_allow_unused_references(true);
    moco.set_markers_global_tracking_weight(10);

    // adjust weights
    MocoWeightSet weights;
    weights.cloneAndAppend(MocoWeight("RASI", 20));
    weights.cloneAndAppend(MocoWeight("LASI", 20));
    weights.cloneAndAppend(MocoWeight("P1", 20));
    weights.cloneAndAppend(MocoWeight("P2", 20));
    weights.cloneAndAppend(MocoWeight("P3", 20));
    weights.cloneAndAppend(MocoWeight("RLEPI", 10));
    weights.cloneAndAppend(MocoWeight("LLEPI", 10));
    weights.cloneAndAppend(MocoWeight("RLMAL", 10));
    weights.cloneAndAppend(MocoWeight("LLMAL", 10));
    weights.cloneAndAppend(MocoWeight("RHEEL", 10));
    weights.cloneAndAppend(MocoWeight("LHEEL", 10));
    weights.cloneAndAppend(MocoWeight("RP5MT", 5));
    weights.cloneAndAppend(MocoWeight("LP5MT", 5));
    weights.cloneAndAppend(MocoWeight("RTOE", 2));
    weights.cloneAndAppend(MocoWeight("LTOE", 2));
    weights.cloneAndAppend(MocoWeight("RSH", 10));
    weights.cloneAndAppend(MocoWeight("LSH", 10));
    weights.cloneAndAppend(MocoWeight("RELB", 5));
    weights.cloneAndAppend(MocoWeight("LELB", 5));
    weights.cloneAndAppend(MocoWeight("RWR", 1));
    weights.cloneAndAppend(MocoWeight("LWR", 1));
    moco.set_markers_weight_set(weights);

    // set simulation time and grid size
    moco.set_initial_time(0.13);
    moco.set_final_time(0.71);
    moco.set_mesh_interval(0.05);

    // solve, visualise and write to file
    //MocoSolution solution = moco.solve(true);
    //solution.write("sandboxReplaceThelenMuscles_marker_tracking_solution.sto");

    return EXIT_SUCCESS;

}
