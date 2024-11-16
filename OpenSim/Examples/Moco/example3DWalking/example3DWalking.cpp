/* -------------------------------------------------------------------------- *
 * OpenSim Moco: example3DWalking.cpp                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2024 Stanford University and the Authors                     *
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

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/VisualizerUtilities.h>
#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Actuators/CoordinateActuator.h>

using namespace OpenSim;



int main() {

    // Load the base model.
    Model model("subject_walk_scaled.osim");
    model.initSystem();


    ExpressionBasedCoordinateForce* ebcf_toes_l = 
        new ExpressionBasedCoordinateForce("mtp_angle_l", "-25.0*q-2.0*qdot");
    model.addComponent(ebcf_toes_l);
    ExpressionBasedCoordinateForce* ebcf_toes_r = 
        new ExpressionBasedCoordinateForce("mtp_angle_r", "-25.0*q-2.0*qdot");
    model.addComponent(ebcf_toes_r);




    

    // VisualizerUtilities::showMotion(feetModel, feetCoordinateReference);

    // Create a tracking simulation to modify the feet kinematics so that the 
    // foot-ground contact model produces more realistic ground reaction forces.
    MocoTrack track;
    track.setModel(ModelProcessor(feetModel));
    track.setStatesReference( 
            TableProcessor(feetCoordinateReference) |
            TabOpLowPassFilter(20));
    track.set_states_global_tracking_weight(1.0);
    MocoWeightSet stateWeightSet;
    stateWeightSet.cloneAndAppend({"/jointset/mtp_r/mtp_angle_r/value", 1e-2});
    stateWeightSet.cloneAndAppend({"/jointset/mtp_r/mtp_angle_r/speed", 1e-2});
    stateWeightSet.cloneAndAppend({"/jointset/mtp_l/mtp_angle_l/value", 1e-2});
    stateWeightSet.cloneAndAppend({"/jointset/mtp_l/mtp_angle_l/speed", 1e-2});
    stateWeightSet.cloneAndAppend({"/jointset/calcn_r/calcn_r_ty/value", 0.0});
    stateWeightSet.cloneAndAppend({"/jointset/calcn_l/calcn_l_ty/value", 0.0});
    track.set_states_weight_set(stateWeightSet);
    track.set_track_reference_position_derivatives(true);
    track.set_apply_tracked_states_to_guess(true);
    const auto& times = feetCoordinateReference.getIndependentColumn();
    track.set_initial_time(times[0]);
    track.set_final_time(times[times.size() - 1]);
    MocoStudy study = track.initialize();
    MocoProblem& problem = study.updProblem();

    // Update the effort weight.
    MocoControlGoal& effort =
            dynamic_cast<MocoControlGoal&>(problem.updGoal("control_effort"));
    effort.setWeight(1.0);

    // Add the contact tracking goal.
    auto* contactTracking = 
            problem.addGoal<MocoContactTrackingGoal>("contact_tracking", 1e-2);
    contactTracking->setExternalLoadsFile("grf_walk.xml");
    MocoContactTrackingGoalGroup leftContactGroup(
            {"contactHeel_l", "contactLateralRearfoot_l", 
             "contactLateralMidfoot_l", "contactMedialMidfoot_l"}, 
             "Left_GRF",
             {"contactLateralToe_l", "contactMedialToe_l"});
    contactTracking->addContactGroup(leftContactGroup);
    MocoContactTrackingGoalGroup rightContactGroup(
            {"contactHeel_r", "contactLateralRearfoot_r", 
             "contactLateralMidfoot_r", "contactMedialMidfoot_r"}, 
             "Right_GRF",
             {"contactLateralToe_r", "contactMedialToe_r"});
    contactTracking->addContactGroup(rightContactGroup);

    // Set coordinate bounds.
    problem.setStateInfoPattern(".*/calcn_.*_r.*/value", {-SimTK::Pi, SimTK::Pi});
    problem.setStateInfoPattern(".*/calcn_.*_t.*/value", {-5.0, 5.0});
    problem.setStateInfoPattern(".*speed", {-50, 50});

    // Configure the solver.
    MocoCasADiSolver& solver = study.updSolver<MocoCasADiSolver>();
    solver.set_num_mesh_intervals(100);
    solver.set_transcription_scheme("legendre-gauss-radau-3");
    solver.set_optim_convergence_tolerance(1e-2);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(3000);

    // Solve!
    // MocoSolution solution = study.solve().unseal();
    // solution.write("feet_tracking_solution.sto");

    // // Extract the ground reaction forces.
    // std::vector<std::string> contact_r = {"contactHeel_r", 
    //         "contactLateralRearfoot_r", "contactLateralMidfoot_r", 
    //         "contactMedialMidfoot_r", "contactLateralToe_r", 
    //         "contactMedialToe_r"};
    // std::vector<std::string> contact_l = {"contactHeel_l", 
    //         "contactLateralRearfoot_l", "contactLateralMidfoot_l", 
    //         "contactMedialMidfoot_l", "contactLateralToe_l", 
    //         "contactMedialToe_l"};
    // TimeSeriesTable externalForcesTableFlat = createExternalLoadsTableForGait(
    //         feetModel, solution, contact_r, contact_l);
    // STOFileAdapter::write(externalForcesTableFlat,
    //         "feet_tracking_solution_ground_reactions.sto");

    // study.visualize((solution));

    return EXIT_SUCCESS;
}