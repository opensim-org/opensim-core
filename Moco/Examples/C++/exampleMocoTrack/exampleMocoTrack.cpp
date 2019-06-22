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

using namespace OpenSim;

void muscleDrivenStateTracking() {

    MocoTrack track;
    track.setName("muscle_driven_state_tracking");

    track.setModel(ModelProcessor("subject_walk_rra_adjusted_armless.osim") |
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
    //track.set_guess_file("muscle_driven_state_tracking_solution.sto");

    //MocoSolution solution = track.solve();
    MocoStudy moco = track.initialize();
    MocoSolution solution = moco.solve();
    moco.visualize(solution);
}


int main() {
    
    muscleDrivenStateTracking();

    return EXIT_SUCCESS;
}
