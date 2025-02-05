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

/// This example demonstrates how to use the MocoTrack tool to solve 3D 
/// walking optimization problems using a foot-ground contact model. 
/// Polynomial functions are used to represent muscle geometry via the
/// FunctionBasedPath class which significantly improves convergence time.
///
/// See the README.txt next to this file for more information about the
/// reference data used in this example.

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/VisualizerUtilities.h>
#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>

using namespace OpenSim;

 // Paths to the contact forces in the model.
static const std::vector<std::string> contactForcesRight = {"/contactHeel_r", 
        "/contactLateralRearfoot_r", "/contactLateralMidfoot_r", 
        "/contactMedialMidfoot_r", "/contactLateralToe_r",  
        "/contactMedialToe_r"};
static const std::vector<std::string> contactForcesLeft = {"/contactHeel_l", 
        "/contactLateralRearfoot_l", "/contactLateralMidfoot_l", 
        "/contactMedialMidfoot_l", "/contactLateralToe_l", 
        "/contactMedialToe_l"};

/// Solve a optimization problem tracking joint kinematics and ground reaction
/// forces using a muscle-driven model with foot-ground contact elements.
void trackWalking(Model model) {
    
    // Modify the model to prepare it for tracking optimization.
    model.initSystem();
    ModelProcessor modelProcessor(model);
    modelProcessor.append(ModOpIgnoreTendonCompliance());
    modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
    modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
    modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
    modelProcessor.append(ModOpReplacePathsWithFunctionBasedPaths(
            "subject_walk_scaled_FunctionBasedPathSet.xml"));

    // Construct the reference kinematics TableProcessor.
    TableProcessor tableProcessor = TableProcessor("coordinates.sto") |
            TabOpUseAbsoluteStateNames() |
            TabOpAppendCoupledCoordinateValues() |
            TabOpAppendCoordinateValueDerivativesAsSpeeds();
    
    // Construct the MocoTrack tool.
    MocoTrack track;
    track.setName("track_walking");
    track.setModel(modelProcessor);
    track.setStatesReference(tableProcessor);
    track.set_states_global_tracking_weight(0.05);
    track.set_control_effort_weight(0.1);
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    track.set_initial_time(0.48);
    track.set_final_time(1.61);
    track.set_mesh_interval(0.02);

    // Update individual state weights.
    MocoWeightSet statesWeightSet;
    // Don't track the veritcal position of the pelvis and only lightly track
    // the speed. Let the optimization determine the vertical position of the
    // model, which will make it easier to find the position of the feet that 
    // leads to the best tracking of the kinematics and ground reaction forces.
    statesWeightSet.cloneAndAppend(
            {"/jointset/ground_pelvis/pelvis_ty/value", 0.0});
    statesWeightSet.cloneAndAppend(
            {"/jointset/ground_pelvis/pelvis_ty/speed", 0.1});
    // Let the toe coordinates be driven by the passive forces in the model.
    statesWeightSet.cloneAndAppend({"/jointset/mtp_r/mtp_angle_r/value", 0.0});
    statesWeightSet.cloneAndAppend({"/jointset/mtp_r/mtp_angle_r/speed", 0.0});
    statesWeightSet.cloneAndAppend({"/jointset/mtp_l/mtp_angle_l/value", 0.0});
    statesWeightSet.cloneAndAppend({"/jointset/mtp_l/mtp_angle_l/speed", 0.0});
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

    // Constrain the states and controls to be periodic.
    auto* periodicityGoal = problem.addGoal<MocoPeriodicityGoal>("periodicity");
    for (const auto& coord : model.getComponentList<Coordinate>()) {
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
    for (const auto& actu : model.getComponentList<Actuator>()) {
        periodicityGoal->addControlPair(actu.getAbsolutePathString());
    }

    // Customize the solver settings.
    // ------------------------------
    auto& solver = study.updSolver<MocoCasADiSolver>();
    // Use the Legnedre-Gauss-Radau transcription scheme, a psuedospectral 
    // method scheme with integration accuracy.
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
    // transcription scheme, it is a good idea to generate a new guess. In this 
    // case, generating a new guess is crucial for the optimization to converge.
    // Always check your initial guess before running an optimization!
    solver.setGuess(solver.createGuess());

    // Solve!
    // ------
    MocoSolution solution = study.solve().unseal();
    solution.write("example3DWalking_track_walking.sto");

    // Print the model.
    Model modelSolution = modelProcessor.process();
    modelSolution.initSystem();
    modelSolution.print("example3DWalking_track_walking_model.osim");

    // Extract the ground reaction forces.
    TimeSeriesTable externalForcesTableFlat = createExternalLoadsTableForGait(
            modelSolution, solution, contactForcesRight, contactForcesLeft);
    STOFileAdapter::write(externalForcesTableFlat,
            "example3DWalking_track_walking_ground_reactions.sto");

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

    // Set minimum muscle controls to 0 (default is 0.01).
    for (auto& muscle : model.updComponentList<Millard2012EquilibriumMuscle>()) {
        muscle.setMinimumActivation(0.0);
        muscle.setMinControl(0.0);
    }

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
    model.finalizeConnections();

    // Add the contact forces to the model.
    ForceSet contactForceSet("subject_walk_scaled_ContactForceSet.xml");
    for (int i = 0; i < contactForceSet.getSize(); ++i) {
        model.addComponent(contactForceSet.get(i).clone());
    }
    model.finalizeConnections();

    // Tracking optimization.
    // ---------------------
    /// Solve a tracking optimization problem using the modified model.
    /// This problem takes ~70 minutes to solve on a machine using a 4.7 GHz
    /// processor with 24 threads.
    trackWalking(model);

    return EXIT_SUCCESS;
}