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


void exampleTrackWalking(Model model) {

    // Create and name an instance of the MocoTrack tool.
    MocoTrack track;
    track.setName("track_walking");

    // Construct a ModelProcessor and set it on the tool. The default
    // muscles in the model are replaced with optimization-friendly
    // DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
    // parameters.
    ModelProcessor modelProcessor(model);
    modelProcessor.append(ModOpIgnoreTendonCompliance());
    modelProcessor.append(ModOpIgnoreActivationDynamics());
    modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
    // Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
    // Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));

    modelProcessor.append(ModOpAddResiduals(250.0, 50.0, 1.0));
    // Use a function-based representation for the muscle paths. This is
    // recommended to speed up convergence, but if you would like to use
    // the original GeometryPath muscle wrapping instead, simply comment out
    // this line. To learn how to create a set of function-based paths for
    // your model, see the example 'examplePolynomialPathFitter.py/.m'.
    modelProcessor.append(ModOpReplacePathsWithFunctionBasedPaths(
            "subject_walk_scaled_FunctionBasedPathSet.xml"));
    track.setModel(modelProcessor);

    // Construct a TableProcessor of the coordinate data and pass it to the 
    // tracking tool. TableProcessors can be used in the same way as
    // ModelProcessors by appending TableOperators to modify the base table.
    // A TableProcessor with no operators, as we have here, simply returns the
    // base table.
    TimeSeriesTable coordinatesUpdated("coordinates_updated.sto");
    // coordinatesUpdated.trim(0.75, 1.0);
    track.setStatesReference(TableProcessor(coordinatesUpdated));
    track.set_apply_tracked_states_to_guess(true);
    track.set_states_global_tracking_weight(10.0);

    // This setting allows extra data columns contained in the states
    // reference that don't correspond to model coordinates.
    track.set_allow_unused_references(true);

    // Since there is only coordinate position data in the states references,
    // this setting is enabled to fill in the missing coordinate speed data
    // using the derivative of splined position data.
    track.set_track_reference_position_derivatives(true);

    MocoWeightSet statesWeightSet;
    // statesWeightSet.cloneAndAppend({"/jointset/ground_pelvis/pelvis_ty/value", 0.0});
    // statesWeightSet.cloneAndAppend({"/jointset/ground_pelvis/pelvis_ty/speed", 0.0});
    statesWeightSet.cloneAndAppend({"/jointset/mtp_r/mtp_angle_r/value", 0.01});
    statesWeightSet.cloneAndAppend({"/jointset/mtp_r/mtp_angle_r/speed", 0.01});
    statesWeightSet.cloneAndAppend({"/jointset/mtp_l/mtp_angle_l/value", 0.01});
    statesWeightSet.cloneAndAppend({"/jointset/mtp_l/mtp_angle_l/speed", 0.01});
    track.set_states_weight_set(statesWeightSet);

    // Initial time, final time, and mesh interval.
    track.set_initial_time(0.48);
    track.set_final_time(1.61);
    track.set_mesh_interval(0.02);

    // Instead of calling solve(), call initialize() to receive a pre-configured
    // MocoStudy object based on the settings above. Use this to customize the
    // problem beyond the MocoTrack interface.
    MocoStudy study = track.initialize();

    // Get a reference to the MocoControlGoal that is added to every MocoTrack
    // problem by default.
    MocoProblem& problem = study.updProblem();
    // MocoControlGoal& effort =
    //     dynamic_cast<MocoControlGoal&>(problem.updGoal("control_effort"));
    // effort.setWeight(0.1);

    // Add a MocoContactTrackingGoal to the problem to track the ground reaction
    // forces.
    auto* contactTracking = problem.addGoal<MocoContactTrackingGoal>(
            "grf_tracking", 1e-6);
    contactTracking->setExternalLoadsFile("grf_walk.xml");

    MocoContactTrackingGoalGroup rightContactGroup(
            {"/contactHeel_r", "/contactLateralRearfoot_r", 
             "/contactLateralMidfoot_r", "/contactMedialMidfoot_r",
             "/contactLateralToe_r", "/contactMedialToe_r"}, 
            "Right_GRF", {"/bodyset/toes_r"});
    contactTracking->addContactGroup(rightContactGroup);
    
    MocoContactTrackingGoalGroup leftContactGroup(
            {"/contactHeel_l", "/contactLateralRearfoot_l", 
             "/contactLateralMidfoot_l", "/contactMedialMidfoot_l",
             "/contactLateralToe_l", "/contactMedialToe_l"}, 
            "Left_GRF", {"/bodyset/toes_l"});
    contactTracking->addContactGroup(leftContactGroup);

    // Constrain the states and controls to be periodic.
    model.initSystem();
    auto* periodicityGoal = problem.addGoal<MocoPeriodicityGoal>("periodicity");
    for (const auto& coord : model.getComponentList<Coordinate>()) {
        if (!IO::EndsWith(coord.getName(), "_tx")) {
            periodicityGoal->addStatePair(coord.getStateVariableNames()[0]);
        }
        periodicityGoal->addStatePair(coord.getStateVariableNames()[1]);
    }
    for (const auto& muscle : model.getComponentList<Muscle>()) {
        // periodicityGoal->addStatePair(muscle.getStateVariableNames()[0]);
        periodicityGoal->addControlPair(muscle.getAbsolutePathString());
    }
    for (const auto& actu : model.getComponentList<Actuator>()) {
        periodicityGoal->addControlPair(actu.getAbsolutePathString());
    }

    // Update the solver tolerances.
    auto& solver = study.updSolver<MocoCasADiSolver>();
    solver.set_transcription_scheme("legendre-gauss-radau-2");
    solver.set_kinematic_constraint_method("Bordalba2023");
    solver.set_optim_convergence_tolerance(1e-2);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(100);

    // Solve!
    MocoSolution solution = study.solve().unseal();
    solution.write("example3DWalking_tracking_solution.sto");

    Model modelSolution = modelProcessor.process();
    modelSolution.initSystem();
    modelSolution.print("example3DWalking_tracking_walking.osim");

    // Visualize the solution.
    study.visualize(solution);
}


int main() {

    // Load the base model.
    Model model("subject_walk_scaled.osim");
    model.initSystem();

    // Add stiffness and damping to the toes.
    ExpressionBasedCoordinateForce* ebcf_toes_l = 
        new ExpressionBasedCoordinateForce("mtp_angle_l", "-25.0*q-2.0*qdot");
    ebcf_toes_l->setName("toe_damping_l");
    model.addForce(ebcf_toes_l);
    ExpressionBasedCoordinateForce* ebcf_toes_r = 
        new ExpressionBasedCoordinateForce("mtp_angle_r", "-25.0*q-2.0*qdot");
    ebcf_toes_r->setName("toe_damping_r");
    model.addForce(ebcf_toes_r);

    // Add CoordinateActuators to the toes.
    CoordinateActuator* ca_toes_l = new CoordinateActuator("mtp_angle_l");
    ca_toes_l->setName("mtp_angle_l_actuator");
    ca_toes_l->setOptimalForce(50);
    ca_toes_l->setMinControl(-1.0);
    ca_toes_l->setMaxControl(1.0);
    model.addForce(ca_toes_l);

    CoordinateActuator* ca_toes_r = new CoordinateActuator("mtp_angle_r");
    ca_toes_r->setName("mtp_angle_r_actuator");
    ca_toes_r->setOptimalForce(50);
    ca_toes_r->setMinControl(-1.0);
    ca_toes_r->setMaxControl(1.0);
    model.addForce(ca_toes_r);

    // Add the contact geometry and forces to the model.
    ContactGeometrySet contactGeometrySet("subject_walk_scaled_ContactGeometrySet.xml");
    for (int i = 0; i < contactGeometrySet.getSize(); ++i) {
        ContactGeometry* contactGeometry = contactGeometrySet.get(i).clone();
        // Raise the ContactSpheres up by 2 cm so that the bottom of the spheres
        // are better aligned with the ground.
        if (contactGeometry->getName() != "floor") {
            SimTK::Vec3& location = contactGeometry->upd_location();
            location[1] += 0.02; 
        }
        model.addContactGeometry(contactGeometry);
    }
    model.finalizeConnections();

    ForceSet contactForceSet("subject_walk_scaled_ContactForceSet.xml");
    for (int i = 0; i < contactForceSet.getSize(); ++i) {
        model.addComponent(contactForceSet.get(i).clone());
    }
    model.finalizeConnections();

    exampleTrackWalking(model);




    

    // VisualizerUtilities::showMotion(feetModel, feetCoordinateReference);

    // Create a tracking simulation to modify the feet kinematics so that the 
    // foot-ground contact model produces more realistic ground reaction forces.
    // MocoTrack track;
    // track.setModel(ModelProcessor(feetModel));
    // track.setStatesReference( 
    //         TableProcessor(feetCoordinateReference) |
    //         TabOpLowPassFilter(20));
    // track.set_states_global_tracking_weight(1.0);
    // MocoWeightSet stateWeightSet;
    // stateWeightSet.cloneAndAppend({"/jointset/mtp_r/mtp_angle_r/value", 1e-2});
    // stateWeightSet.cloneAndAppend({"/jointset/mtp_r/mtp_angle_r/speed", 1e-2});
    // stateWeightSet.cloneAndAppend({"/jointset/mtp_l/mtp_angle_l/value", 1e-2});
    // stateWeightSet.cloneAndAppend({"/jointset/mtp_l/mtp_angle_l/speed", 1e-2});
    // stateWeightSet.cloneAndAppend({"/jointset/calcn_r/calcn_r_ty/value", 0.0});
    // stateWeightSet.cloneAndAppend({"/jointset/calcn_l/calcn_l_ty/value", 0.0});
    // track.set_states_weight_set(stateWeightSet);
    // track.set_track_reference_position_derivatives(true);
    // track.set_apply_tracked_states_to_guess(true);
    // const auto& times = feetCoordinateReference.getIndependentColumn();
    // track.set_initial_time(times[0]);
    // track.set_final_time(times[times.size() - 1]);
    // MocoStudy study = track.initialize();
    // MocoProblem& problem = study.updProblem();

    // // Update the effort weight.
    // MocoControlGoal& effort =
    //         dynamic_cast<MocoControlGoal&>(problem.updGoal("control_effort"));
    // effort.setWeight(1.0);

    // // Add the contact tracking goal.
    // auto* contactTracking = 
    //         problem.addGoal<MocoContactTrackingGoal>("contact_tracking", 1e-2);
    // contactTracking->setExternalLoadsFile("grf_walk.xml");
    // MocoContactTrackingGoalGroup leftContactGroup(
    //         {"contactHeel_l", "contactLateralRearfoot_l", 
    //          "contactLateralMidfoot_l", "contactMedialMidfoot_l"}, 
    //          "Left_GRF",
    //          {"contactLateralToe_l", "contactMedialToe_l"});
    // contactTracking->addContactGroup(leftContactGroup);
    // MocoContactTrackingGoalGroup rightContactGroup(
    //         {"contactHeel_r", "contactLateralRearfoot_r", 
    //          "contactLateralMidfoot_r", "contactMedialMidfoot_r"}, 
    //          "Right_GRF",
    //          {"contactLateralToe_r", "contactMedialToe_r"});
    // contactTracking->addContactGroup(rightContactGroup);

    // // Set coordinate bounds.
    // problem.setStateInfoPattern(".*/calcn_.*_r.*/value", {-SimTK::Pi, SimTK::Pi});
    // problem.setStateInfoPattern(".*/calcn_.*_t.*/value", {-5.0, 5.0});
    // problem.setStateInfoPattern(".*speed", {-50, 50});

    // // Configure the solver.
    // MocoCasADiSolver& solver = study.updSolver<MocoCasADiSolver>();
    // solver.set_num_mesh_intervals(100);
    // solver.set_transcription_scheme("legendre-gauss-radau-3");
    // solver.set_optim_convergence_tolerance(1e-2);
    // solver.set_optim_constraint_tolerance(1e-4);
    // solver.set_optim_max_iterations(3000);

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