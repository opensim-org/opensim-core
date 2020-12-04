/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testMocoInverse.cpp                                          *
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

#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Moco/osimMoco.h>

#define CATCH_CONFIG_MAIN
#include "Testing.h"

using namespace OpenSim;

TEST_CASE("MocoTrack interface") {
    MocoTrack track;
    track.setModel(ModelProcessor("testMocoTrack_subject01.osim"));

    SECTION("apply_tracked_states_to_guess() true, but no states reference") {
        track.set_apply_tracked_states_to_guess(true);
        CHECK_THROWS_WITH(track.initialize(),
            Catch::Contains("Property 'apply_tracked_states_to_guess' was "
                    "enabled, but no states reference data was provided."));
    }
    track.set_apply_tracked_states_to_guess(false);

    SECTION("negative control effort weight") {
        track.set_control_effort_weight(-1);
        CHECK_THROWS_WITH(track.initialize(),
            Catch::Contains("Expected a non-negative control effort weight, " 
                    "but got a weight with value"));
    }
}

TEST_CASE("MocoTrack gait10dof18musc", "[casadi]") {

    MocoTrack track;

    track.setModel(ModelProcessor("testMocoTrack_subject01.osim") |
            ModOpRemoveMuscles() | ModOpAddReserves(100) |
            ModOpAddExternalLoads("walk_gait1018_subject01_grf.xml"));
    track.setStatesReference(
            TableProcessor("walk_gait1018_state_reference.mot") |
            TabOpLowPassFilter(6));
    track.set_initial_time(0.01);
    track.set_final_time(1.3);
    track.print("testMocoTrack_setup.xml");

    MocoSolution solution = track.solve();
    //solution.write("testMocoTrackGait10dof18musc_solution.sto");

    const auto actual = solution.getControlsTrajectory();
    MocoTrajectory std("std_testMocoTrackGait10dof18musc_solution.sto");
    const auto expected = std.getControlsTrajectory();
    CHECK(std.compareContinuousVariablesRMS(
            solution, {{"controls",{}}}) < 1e-2);
}
