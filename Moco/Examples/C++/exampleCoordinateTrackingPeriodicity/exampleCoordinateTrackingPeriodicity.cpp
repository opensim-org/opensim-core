/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleCoordinateTrackingSymmetry.cpp                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-19 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Antoine Falisse                                                 *
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
void testCoordinateTracking_CoordinateActuators() {
    // Create a MocoTrack
    MocoTrack track;
    track.setName("coordinateTracking_CoordinateActuators");
    // Set model
    ModelProcessor modelprocessor = ModelProcessor(
            "gait10dof18musc_CoordinateActuators.osim");
    track.setModel(modelprocessor);
    // Set experimental coordinate values to track
    track.setStatesReference(TableProcessor("referenceCoordinate.sto") |
            TabOpLowPassFilter(6));
    track.set_states_global_tracking_weight(10.0);
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    // Set guess
    track.set_apply_tracked_states_to_guess(true);
    // Set time bounds
    track.set_initial_time(0.0);
    track.set_final_time(0.47008941);
    // Initialize study
    MocoStudy moco = track.initialize();
    // Set solver settings
    MocoCasADiSolver& solver = moco.updSolver<MocoCasADiSolver>();
    solver.set_num_mesh_points(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(100000);
    solver.set_parallel(8);
    // Update problem
    MocoProblem& problem = moco.updProblem();
    // Add periodicity goal
    auto* periodicityGoal =
            problem.addGoal<MocoPeriodicityGoal>("periodicityGoal");
    // coordinate values
    MocoPeriodicityGoalPair pair_pelvis_tilt_value;
    pair_pelvis_tilt_value.set_first(
            "/jointset/groundPelvis/pelvis_tilt/value");
    pair_pelvis_tilt_value.set_second(
            "/jointset/groundPelvis/pelvis_tilt/value");
    MocoPeriodicityGoalPair pair_pelvis_ty_value;
    pair_pelvis_ty_value.set_first("/jointset/groundPelvis/pelvis_ty/value");
    pair_pelvis_ty_value.set_second("/jointset/groundPelvis/pelvis_ty/value");
    MocoPeriodicityGoalPair pair_hip_1_value;
    pair_hip_1_value.set_first("/jointset/hip_l/hip_flexion_l/value");
    pair_hip_1_value.set_second("/jointset/hip_r/hip_flexion_r/value");
    MocoPeriodicityGoalPair pair_hip_2_value;
    pair_hip_2_value.set_first("/jointset/hip_r/hip_flexion_r/value");
    pair_hip_2_value.set_second("/jointset/hip_l/hip_flexion_l/value");
    MocoPeriodicityGoalPair pair_knee_1_value;
    pair_knee_1_value.set_first("/jointset/knee_l/knee_angle_l/value");
    pair_knee_1_value.set_second("/jointset/knee_r/knee_angle_r/value");
    MocoPeriodicityGoalPair pair_knee_2_value;
    pair_knee_2_value.set_first("/jointset/knee_r/knee_angle_r/value");
    pair_knee_2_value.set_second("/jointset/knee_l/knee_angle_l/value");
    MocoPeriodicityGoalPair pair_ankle_1_value;
    pair_ankle_1_value.set_first("/jointset/ankle_l/ankle_angle_l/value");
    pair_ankle_1_value.set_second("/jointset/ankle_r/ankle_angle_r/value");
    MocoPeriodicityGoalPair pair_ankle_2_value;
    pair_ankle_2_value.set_first("/jointset/ankle_r/ankle_angle_r/value");
    pair_ankle_2_value.set_second("/jointset/ankle_l/ankle_angle_l/value");
    MocoPeriodicityGoalPair pair_lumbar_value;
    pair_lumbar_value.set_first("/jointset/lumbar/lumbar/value");
    pair_lumbar_value.set_second("/jointset/lumbar/lumbar/value");
    // coordinate speeds
    MocoPeriodicityGoalPair pair_pelvis_tilt_speed;
    pair_pelvis_tilt_speed.set_first(
            "/jointset/groundPelvis/pelvis_tilt/speed");
    pair_pelvis_tilt_speed.set_second(
            "/jointset/groundPelvis/pelvis_tilt/speed");
    MocoPeriodicityGoalPair pair_pelvis_tx_speed;
    pair_pelvis_tx_speed.set_first("/jointset/groundPelvis/pelvis_tx/speed");
    pair_pelvis_tx_speed.set_second("/jointset/groundPelvis/pelvis_tx/speed");
    MocoPeriodicityGoalPair pair_pelvis_ty_speed;
    pair_pelvis_ty_speed.set_first("/jointset/groundPelvis/pelvis_ty/speed");
    pair_pelvis_ty_speed.set_second("/jointset/groundPelvis/pelvis_ty/speed");
    MocoPeriodicityGoalPair pair_hip_1_speed;
    pair_hip_1_speed.set_first("/jointset/hip_l/hip_flexion_l/speed");
    pair_hip_1_speed.set_second("/jointset/hip_r/hip_flexion_r/speed");
    MocoPeriodicityGoalPair pair_hip_2_speed;
    pair_hip_2_speed.set_first("/jointset/hip_r/hip_flexion_r/speed");
    pair_hip_2_speed.set_second("/jointset/hip_l/hip_flexion_l/speed");
    MocoPeriodicityGoalPair pair_knee_1_speed;
    pair_knee_1_speed.set_first("/jointset/knee_l/knee_angle_l/speed");
    pair_knee_1_speed.set_second("/jointset/knee_r/knee_angle_r/speed");
    MocoPeriodicityGoalPair pair_knee_2_speed;
    pair_knee_2_speed.set_first("/jointset/knee_r/knee_angle_r/speed");
    pair_knee_2_speed.set_second("/jointset/knee_l/knee_angle_l/speed");
    MocoPeriodicityGoalPair pair_ankle_1_speed;
    pair_ankle_1_speed.set_first("/jointset/ankle_l/ankle_angle_l/speed");
    pair_ankle_1_speed.set_second("/jointset/ankle_r/ankle_angle_r/speed");
    MocoPeriodicityGoalPair pair_ankle_2_speed;
    pair_ankle_2_speed.set_first("/jointset/ankle_r/ankle_angle_r/speed");
    pair_ankle_2_speed.set_second("/jointset/ankle_l/ankle_angle_l/speed");
    MocoPeriodicityGoalPair pair_lumbar_speed;
    pair_lumbar_speed.set_first("/jointset/lumbar/lumbar/speed");
    pair_lumbar_speed.set_second("/jointset/lumbar/lumbar/speed");
    // coordinate actuator controls
    MocoPeriodicityGoalPair pair_hip_1_act;
    pair_hip_1_act.set_first("/hipAct_l");
    pair_hip_1_act.set_second("/hipAct_r");
    MocoPeriodicityGoalPair pair_hip_2_act;
    pair_hip_2_act.set_first("/hipAct_r");
    pair_hip_2_act.set_second("/hipAct_l");
    MocoPeriodicityGoalPair pair_knee_1_act;
    pair_knee_1_act.set_first("/kneeAct_l");
    pair_knee_1_act.set_second("/kneeAct_r");
    MocoPeriodicityGoalPair pair_knee_2_act;
    pair_knee_2_act.set_first("/kneeAct_r");
    pair_knee_2_act.set_second("/kneeAct_l");
    MocoPeriodicityGoalPair pair_ankle_1_act;
    pair_ankle_1_act.set_first("/ankleAct_l");
    pair_ankle_1_act.set_second("/ankleAct_r");
    MocoPeriodicityGoalPair pair_ankle_2_act;
    pair_ankle_2_act.set_first("/ankleAct_r");
    pair_ankle_2_act.set_second("/ankleAct_l");
    MocoPeriodicityGoalPair pair_lumbar_act;
    pair_lumbar_act.set_first("/lumbarAct");
    pair_lumbar_act.set_second("/lumbarAct");
    // Append the state pairs
    periodicityGoal->append_state_pair(pair_pelvis_tilt_value);
    periodicityGoal->append_state_pair(pair_pelvis_ty_value);
    periodicityGoal->append_state_pair(pair_hip_1_value);
    periodicityGoal->append_state_pair(pair_hip_2_value);
    periodicityGoal->append_state_pair(pair_knee_1_value);
    periodicityGoal->append_state_pair(pair_knee_2_value);
    periodicityGoal->append_state_pair(pair_ankle_1_value);
    periodicityGoal->append_state_pair(pair_ankle_2_value);
    periodicityGoal->append_state_pair(pair_lumbar_value);
    periodicityGoal->append_state_pair(pair_pelvis_tilt_speed);
    periodicityGoal->append_state_pair(pair_pelvis_tx_speed);
    periodicityGoal->append_state_pair(pair_pelvis_ty_speed);
    periodicityGoal->append_state_pair(pair_hip_1_speed);
    periodicityGoal->append_state_pair(pair_hip_2_speed);
    periodicityGoal->append_state_pair(pair_knee_1_speed);
    periodicityGoal->append_state_pair(pair_knee_2_speed);
    periodicityGoal->append_state_pair(pair_ankle_1_speed);
    periodicityGoal->append_state_pair(pair_ankle_2_speed);
    periodicityGoal->append_state_pair(pair_lumbar_speed);
    // Append the control pairs
    periodicityGoal->append_control_pair(pair_hip_1_act);
    periodicityGoal->append_control_pair(pair_hip_2_act);
    periodicityGoal->append_control_pair(pair_knee_1_act);
    periodicityGoal->append_control_pair(pair_knee_2_act);
    periodicityGoal->append_control_pair(pair_ankle_1_act);
    periodicityGoal->append_control_pair(pair_ankle_2_act);
    periodicityGoal->append_control_pair(pair_lumbar_act);
    // Add effort goal
    auto* effortGoal = problem.addGoal<MocoControlGoal>("effortGoal");
    effortGoal->setWeight(10);
    // Adjust bounds
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
            {-20*SimTK::Pi/180,-10*SimTK::Pi/180});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0,1});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",
            {0.75,1.25});
    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
            {-10*SimTK::Pi/180,60*SimTK::Pi/180});
    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",
            {-50*SimTK::Pi/180,0});
    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
            {-50*SimTK::Pi/180,0});
    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
            {-15*SimTK::Pi/180,25*SimTK::Pi/180});
    problem.setStateInfo("/jointset/lumbar/lumbar/value",
            {0,20*SimTK::Pi/180});
    // Solve problem
    MocoSolution solution = moco.solve();
}

int main() {
   testCoordinateTracking_CoordinateActuators();
}
