/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleMocoTrack.cpp                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Common/TRCFileAdapter.h>

using namespace OpenSim;

void torqueDrivenMarkerTracking() {
    
    MocoTrack track;
    track.setName("torque_driven_marker_tracking");
    track.setModel(ModelProcessor("subject_walk_rra_adjusted_armless.osim") |
        ModOpReplaceJointsWithWelds({"mtp_l", "mtp_r"}) |
        ModOpAddExternalLoads("grf_walk.xml") |
        ModOpRemoveMuscles() |
        ModOpAddReserves(250));

    TimeSeriesTableVec3 markers = 
            TRCFileAdapter::read("motion_capture_walk.trc");
    TimeSeriesTable markersFlat = markers.flatten();
    const SimTK::Real scale = 0.001;
    markersFlat.updMatrix() *= scale;
    track.setMarkersReference(TableProcessor(markersFlat) |
        TabOpLowPassFilter(6));
    track.set_allow_unused_references(true);

    track.set_initial_time(0.81);
    track.set_final_time(1.65);
    track.set_mesh_interval(0.05);    

    MocoStudy moco = track.initialize();

    MocoProblem& problem = moco.updProblem();
    MocoControlCost& effort = 
        dynamic_cast<MocoControlCost&>(problem.updCost("control_effort"));
    
    double residualWeight = 100;
    effort.setWeight("/forceset/reserve_jointset_ground_pelvis_pelvis_tilt",
            residualWeight);
    effort.setWeight("/forceset/reserve_jointset_ground_pelvis_pelvis_list",
            residualWeight);
    effort.setWeight("/forceset/reserve_jointset_ground_pelvis_pelvis_rotation",
            residualWeight);
    effort.setWeight("/forceset/reserve_jointset_ground_pelvis_pelvis_tx",
            residualWeight);
    effort.setWeight("/forceset/reserve_jointset_ground_pelvis_pelvis_ty",
            residualWeight);
    effort.setWeight("/forceset/reserve_jointset_ground_pelvis_pelvis_tz",
            residualWeight);

    MocoSolution solution = moco.solve();
    moco.visualize(solution);

}

void muscleDrivenStateTracking() {

    MocoTrack track;
    track.setName("muscle_driven_state_tracking");

    track.setModel(ModelProcessor("subject_walk_rra_adjusted_armless.osim") |
        //ModOpReplaceJointsWithWelds({"/jointset/mtp_l", "/jointset/mtp_r"}) |
        ModOpAddExternalLoads("grf_walk.xml") |
        ModOpAddReserves(5) |
        ModOpReplaceMusclesWithDeGrooteFregly2016() |
        ModOpIgnorePassiveFiberForces() |
        ModOpScaleActiveFiberForceCurveWidth(1.5));

    track.setStatesReference(TableProcessor("coordinates_rra_adjusted.sto") | 
        TabOpLowPassFilter(6));
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    track.set_initial_time(0.81);
    track.set_final_time(1.65);
    track.set_mesh_interval(0.075);

    MocoSolution solution = track.solve(true);

}


int main() {
    
    torqueDrivenMarkerTracking();
    muscleDrivenStateTracking();

    return EXIT_SUCCESS;
}
