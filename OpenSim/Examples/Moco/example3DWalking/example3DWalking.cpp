/* -------------------------------------------------------------------------- *
 * OpenSim Moco: example3DWalking.cpp                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2025 Stanford University and the Authors                     *
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
#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>

using namespace OpenSim;

/// This example demonstrates how to solve 3D walking optimization problems 
/// using a foot-ground contact model. Polynomial functions are used to 
/// represent muscle geometry via the FunctionBasedPath class which 
/// significantly improves convergence time.
///
/// See the README.txt next to this file for more information about the
/// reference data used in this example.

/// Construct a MocoStudy to track joint kinematics and ground reaction forces 
/// using a torque-driven or muscle-driven model with foot-ground contact 
/// elements. The objective function weights were chosen such the optimized 
/// objective value falls roughly in the range [0.1, 10], which generally 
/// improves convergence.
void runTrackingStudy(Model model, bool muscleDriven) {

     // Paths to the contact forces in the model.
    const std::vector<std::string> contactForcesRight = {
            "/contactHeel_r", "/contactLateralRearfoot_r", 
            "/contactLateralMidfoot_r", "/contactMedialMidfoot_r", 
            "/contactLateralToe_r", "/contactMedialToe_r"};
    const std::vector<std::string> contactForcesLeft = {
            "/contactHeel_l", "/contactLateralRearfoot_l", 
            "/contactLateralMidfoot_l", "/contactMedialMidfoot_l", 
            "/contactLateralToe_l", "/contactMedialToe_l"};

    // Configure study-specific settings.
    std::string study_name;
    if (muscleDriven) {
        study_name = "muscle_driven_tracking";
    } else {
        study_name = "torque_driven_tracking";

        // Add weak CoordinateActuators to the toes. For the torque-driven 
        // simulation, we do not want ModOpAddReserves() to add strong 
        // actuators to the toes, since the toes will have no active actuation
        // in the muscle-driven problem.
        CoordinateActuator* ca_toes_l = new CoordinateActuator("mtp_angle_l");
        ca_toes_l->setName("mtp_angle_l_actuator");
        ca_toes_l->setOptimalForce(10);
        ca_toes_l->setMinControl(-1.0);
        ca_toes_l->setMaxControl(1.0);
        model.addForce(ca_toes_l);

        CoordinateActuator* ca_toes_r = new CoordinateActuator("mtp_angle_r");
        ca_toes_r->setName("mtp_angle_r_actuator");
        ca_toes_r->setOptimalForce(10);
        ca_toes_r->setMinControl(-1.0);
        ca_toes_r->setMaxControl(1.0);
        model.addForce(ca_toes_r);
    }
    
    // Modify the model to prepare it for tracking optimization.
    model.initSystem();
    ModelProcessor modelProcessor(model);
    if (muscleDriven) {
        modelProcessor.append(ModOpIgnoreTendonCompliance());
        modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
        modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
        modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
        modelProcessor.append(ModOpReplacePathsWithFunctionBasedPaths(
                "subject_walk_scaled_FunctionBasedPathSet.xml"));
    } else {
        modelProcessor.append(ModOpRemoveMuscles());
        modelProcessor.append(ModOpAddReserves(500, 1.0, true, true));
    }

    // Construct the reference kinematics TableProcessor.
    TableProcessor tableProcessor = TableProcessor("coordinates.sto") |
            TabOpUseAbsoluteStateNames() |
            TabOpAppendCoupledCoordinateValues() |
            TabOpAppendCoordinateValueDerivativesAsSpeeds();
    
    // Construct the MocoTrack tool.
    MocoTrack track;
    track.setName(study_name);
    track.setModel(modelProcessor);
    track.setStatesReference(tableProcessor);
    track.set_states_global_tracking_weight(0.05);
    track.set_control_effort_weight(0.1);
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    track.set_initial_time(0.48);
    track.set_final_time(1.61);
    track.set_mesh_interval(0.02);

    // Don't track the veritcal position of the pelvis and only lightly track
    // the speed. Let the optimization determine the vertical position of the
    // model, which will make it easier to find the position of the feet that 
    // leads to the best tracking of the kinematics and ground reaction forces.
    MocoWeightSet statesWeightSet;
    statesWeightSet.cloneAndAppend(
            {"/jointset/ground_pelvis/pelvis_ty/value", 0.0});
    statesWeightSet.cloneAndAppend(
            {"/jointset/ground_pelvis/pelvis_ty/speed", 0.1});
    track.set_states_weight_set(statesWeightSet);

    // Get the underlying MocoStudy.
    MocoStudy study = track.initialize();
    MocoProblem& problem = study.updProblem();

    // Add a MocoContactTrackingGoal to the problem to track the ground reaction
    // forces.
    auto* contactTracking = problem.addGoal<MocoContactTrackingGoal>(
            "grf_tracking", 5e-3);
    contactTracking->setExternalLoadsFile("grf_walk.xml");
    MocoContactTrackingGoalGroup rightContactGroup(contactForcesRight, 
            "Right_GRF", {"/bodyset/toes_r"});
    contactTracking->addContactGroup(rightContactGroup);
    MocoContactTrackingGoalGroup leftContactGroup(contactForcesLeft, 
            "Left_GRF", {"/bodyset/toes_l"});
    contactTracking->addContactGroup(leftContactGroup);  

    // Constrain the initial states to be close to the reference.
    TimeSeriesTable coordinatesUpdated = tableProcessor.process(&model);
    int index = (int)coordinatesUpdated.getNearestRowIndexForTime(0.48);
    const auto& labels = coordinatesUpdated.getColumnLabels();
    for (const auto& label : labels) {
        const auto& value = coordinatesUpdated.getDependentColumn(label);        
        double lower = 0.0;
        double upper = 0.0;
        if (label.find("/speed") != std::string::npos) {
            lower = value[index] - 0.1;
            upper = value[index] + 0.1;
        } else {
            lower = value[index] - 0.05;
            upper = value[index] + 0.05;
        }
        problem.setStateInfo(label, {}, {lower, upper});
    }

    // Constrain the states and controls to be periodic.
    if (muscleDriven) {
        auto* periodicityGoal = 
                problem.addGoal<MocoPeriodicityGoal>("periodicity");
        for (const auto& coord : model.getComponentList<Coordinate>()) {
            // Exclude the knee_angle_l/r_beta coordinates from the periodicity
            // constraint because they are coupled to the knee_angle_l/r
            // coordinates.
            if (IO::EndsWith(coord.getName(), "_beta")) { continue; }
            if (!IO::EndsWith(coord.getName(), "_tx")) {
                periodicityGoal->addStatePair(coord.getStateVariableNames()[0]);
            }
            periodicityGoal->addStatePair(coord.getStateVariableNames()[1]);
        }
        for (const auto& muscle : model.getComponentList<Muscle>()) {
            periodicityGoal->addStatePair(muscle.getStateVariableNames()[0]);
            periodicityGoal->addControlPair(muscle.getAbsolutePathString());
        }
        for (const auto& actu : model.getComponentList<CoordinateActuator>()) {
            periodicityGoal->addControlPair(actu.getAbsolutePathString());
        }
    }

    // Customize the solver settings.
    // ------------------------------
    auto& solver = study.updSolver<MocoCasADiSolver>();
    // Use the Legendre-Gauss-Radau transcription scheme, a psuedospectral 
    // scheme with high integration accuracy.
    solver.set_transcription_scheme("legendre-gauss-radau-3");
    // Use the Bordalba et al. (2023) kinematic constraint method.
    solver.set_kinematic_constraint_method("Bordalba2023");
    // Set the solver's convergence and constraint tolerances.
    solver.set_optim_convergence_tolerance(1e-2);
    solver.set_optim_constraint_tolerance(1e-4);
    // We've updated the MocoProblem, so call resetProblem() to pass the updated
    // problem to the solver.
    solver.resetProblem(problem);
    // When MocoTrack::initialize() is called, the solver is created with a
    // default guess. Since we've updated the problem and changed the 
    // transcription scheme, it is a good idea to generate a new guess. If 
    // solving the muscle-driven problem, use the solution from the 
    // torque-driven problem as the initial guess.
    MocoTrajectory guess = solver.createGuess();
    std::string torqueDrivenSolutionFile =
            "example3DWalking_torque_driven_tracking_solution.sto";
    if (muscleDriven && IO::FileExists(torqueDrivenSolutionFile)) {
        MocoTrajectory initialGuess(torqueDrivenSolutionFile);
        guess.insertStatesTrajectory(initialGuess.exportToStatesTable(), true);
        TimeSeriesTable controls = guess.exportToControlsTable();
        controls.updMatrix().setToZero();
        guess.insertControlsTrajectory(controls, true);
    }
    solver.setGuess(guess);

    // Solve!
    // ------
    MocoSolution solution = study.solve();
    solution.write(fmt::format("example3DWalking_{}_solution.sto", study_name));

    // Print the model.
    Model modelSolution = modelProcessor.process();
    modelSolution.initSystem();
    modelSolution.print(fmt::format(
            "example3DWalking_{}_model.osim", study_name));

    // Extract the ground reaction forces.
    TimeSeriesTable externalForcesTableFlat = createExternalLoadsTableForGait(
            modelSolution, solution, contactForcesRight, contactForcesLeft);
    STOFileAdapter::write(externalForcesTableFlat,
            fmt::format("example3DWalking_{}_ground_reactions.sto", study_name));

    // Visualize the solution.
    study.visualize(solution);
}

int main() {

    /// Model preparation.
    /// -----------------
    /// Update the model to prepare it for tracking optimization. The default 
    /// minimimum muscle excitations and activations are set to 0; stiffness, 
    /// damping, and light torque actuation are added to the toes; and contact 
    /// geometry is added to the foot bodies in the model. The height of the 
    /// contact geometry elements are adjusted to better align with the ground.

    // Load the base model.
    Model model("subject_walk_scaled.osim");
    model.initSystem();

    // Set minimum muscle controls and activations to 0 (default is 0.01).
    for (auto& muscle : model.updComponentList<Millard2012EquilibriumMuscle>()) {
        muscle.setMinimumActivation(0.0);
        muscle.setMinControl(0.0);
    }

    // Add stiffness and damping to the joints. Toe stiffness and damping values
    // are based on Falisse et al. (2022), "Modeling toes contributes to 
    // realistic stance knee mechanics in three-dimensional predictive 
    // simulations of walking."
    ForceSet expressionBasedForceSet(
            "subject_walk_scaled_ExpressionBasedCoordinateForceSet.xml");
    for (int i = 0; i < expressionBasedForceSet.getSize(); ++i) {
        model.addComponent(expressionBasedForceSet.get(i).clone());
    }

    // Add the contact geometry to the model.
    ContactGeometrySet contactGeometrySet(
            "subject_walk_scaled_ContactGeometrySet.xml");
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

    // Add the contact forces to the model.
    ForceSet contactForceSet("subject_walk_scaled_ContactForceSet.xml");
    for (int i = 0; i < contactForceSet.getSize(); ++i) {
        model.addComponent(contactForceSet.get(i).clone());
    }
    model.finalizeConnections();

    /// Tracking optimization.
    /// ---------------------
    /// Solve tracking optimization problems using the modified model. The 
    /// convergence times below were estimated on a machine using a processor 
    /// with 4.7 GHz base clock speed and 12 CPU cores (24 threads).

    // Solve a torque-driven tracking problem to create a kinematic 
    // trajectory that is consistent with the ground reaction forces. 
    // This problem takes ~10 minutes to solve.
    runTrackingStudy(model, false);

    // Solve a muscle-driven tracking problem using the kinematic trajectory
    // from the torque-driven problem as the initial guess.
    // This problem takes ~35 minutes to solve.
    runTrackingStudy(model, true);

    return EXIT_SUCCESS;
}