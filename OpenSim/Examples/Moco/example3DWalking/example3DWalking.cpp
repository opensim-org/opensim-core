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
///  - The first problem demonstrates how to track kinematics and ground
///    reaction forces using a torque-driven model.
///  - The second problem uses the solution from the first problem as an initial
///    guess to solve a tracking optimization using a muscle-driven model.
/// 
/// See the README.txt next to this file for more information.

// #include <OpenSim/Moco/osimMoco.h>
// #include <OpenSim/Common/STOFileAdapter.h>
// #include <OpenSim/Simulation/VisualizerUtilities.h>
// #include <OpenSim/Actuators/ModelOperators.h>
// #include <OpenSim/Actuators/CoordinateActuator.h>
// #include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>

// using namespace OpenSim;

// // Paths to the contact forces in the model.
// static const std::vector<std::string> contactForcesRight = {"/contactHeel_r", 
//         "/contactLateralRearfoot_r", "/contactLateralMidfoot_r", 
//         "/contactMedialMidfoot_r", "/contactLateralToe_r", 
//         "/contactMedialToe_r"};
// static const std::vector<std::string> contactForcesLeft = {"/contactHeel_l", 
//         "/contactLateralRearfoot_l", "/contactLateralMidfoot_l", 
//         "/contactMedialMidfoot_l", "/contactLateralToe_l", 
//         "/contactMedialToe_l"};

// /// This helper function loads the base model from file and makes the necessary
// /// modifications to prepare it for tracking optimization. The default minimimum
// /// muscle excitations and activations are set to 0; stiffness, damping, and 
// /// light torque actuation are added to the toes; and contact geometry is added
// /// to the foot bodies in the model. The height of the contact geometry elements
// /// are adjusted to better align with the ground.
// Model loadAndUpdateModel() {

//     // Load the base model.
//     Model model("subject_walk_scaled.osim");
//     model.initSystem();

//     // Set minimum muscle controls to 0 (default is 0.01).
//     for (auto& muscle : model.updComponentList<Millard2012EquilibriumMuscle>()) {
//         muscle.setMinimumActivation(0.0);
//         muscle.setMinControl(0.0);
//     }

//     // Add stiffness and damping to the toes.
//     ExpressionBasedCoordinateForce* ebcf_toes_l = 
//         new ExpressionBasedCoordinateForce("mtp_angle_l", "-25.0*q-2.0*qdot");
//     ebcf_toes_l->setName("toe_damping_l");
//     model.addForce(ebcf_toes_l);
//     ExpressionBasedCoordinateForce* ebcf_toes_r = 
//         new ExpressionBasedCoordinateForce("mtp_angle_r", "-25.0*q-2.0*qdot");
//     ebcf_toes_r->setName("toe_damping_r");
//     model.addForce(ebcf_toes_r);

//     // Add CoordinateActuators to the toes.
//     CoordinateActuator* ca_toes_l = new CoordinateActuator("mtp_angle_l");
//     ca_toes_l->setName("mtp_angle_l_actuator");
//     ca_toes_l->setOptimalForce(50);
//     ca_toes_l->setMinControl(-1.0);
//     ca_toes_l->setMaxControl(1.0);
//     model.addForce(ca_toes_l);

//     CoordinateActuator* ca_toes_r = new CoordinateActuator("mtp_angle_r");
//     ca_toes_r->setName("mtp_angle_r_actuator");
//     ca_toes_r->setOptimalForce(50);
//     ca_toes_r->setMinControl(-1.0);
//     ca_toes_r->setMaxControl(1.0);
//     model.addForce(ca_toes_r);

//     // Add the contact geometry and forces to the model.
//     ContactGeometrySet contactGeometrySet(
//             "subject_walk_scaled_ContactGeometrySet.xml");
//     for (int i = 0; i < contactGeometrySet.getSize(); ++i) {
//         ContactGeometry* contactGeometry = contactGeometrySet.get(i).clone();
//         // Raise the ContactSpheres up by 2 cm so that the bottom of the spheres
//         // are better aligned with the ground.
//         if (contactGeometry->getName() != "floor") {
//             SimTK::Vec3& location = contactGeometry->upd_location();
//             location[1] += 0.02; 
//         }
//         model.addContactGeometry(contactGeometry);
//     }
//     model.finalizeConnections();

//     ForceSet contactForceSet("subject_walk_scaled_ContactForceSet.xml");
//     for (int i = 0; i < contactForceSet.getSize(); ++i) {
//         model.addComponent(contactForceSet.get(i).clone());
//     }
//     model.finalizeConnections();

//     return model;
// }

// // This helper function constructs a base MocoTrack instance given a model 
// // and reference kinematics. 
// MocoStudy constructContactTrackingStudy(const std::string& studyName, 
//         ModelProcessor modelProcessor,
//         TableProcessor tableProcessor) {

//     // Construct the MocoTrack tool.
//     MocoTrack track;
//     track.setName(studyName);
//     track.setModel(modelProcessor);
//     track.setStatesReference(tableProcessor);
//     track.set_states_global_tracking_weight(0.1);
//     track.set_allow_unused_references(true);
//     track.set_track_reference_position_derivatives(true);
//     track.set_initial_time(0.48);
//     track.set_final_time(1.61);
//     track.set_mesh_interval(0.02);

//     // Update individual state weights.
//     MocoWeightSet statesWeightSet;
//     // Don't track the veritcal position of the pelvis and only lightly track
//     // the speed. Let the optimization determine the vertical position of the
//     // model, which will make it easier to find the position of the feet that 
//     // leads to the best tracking of the kinematics and ground reaction forces.
//     statesWeightSet.cloneAndAppend(
//             {"/jointset/ground_pelvis/pelvis_ty/value", 0.0});
//     statesWeightSet.cloneAndAppend(
//             {"/jointset/ground_pelvis/pelvis_ty/speed", 0.1});
//     // Let the toe coordinates be driven by the passive forces in the model.
//     statesWeightSet.cloneAndAppend({"/jointset/mtp_r/mtp_angle_r/value", 0});
//     statesWeightSet.cloneAndAppend({"/jointset/mtp_r/mtp_angle_r/speed", 0});
//     statesWeightSet.cloneAndAppend({"/jointset/mtp_l/mtp_angle_l/value", 0});
//     statesWeightSet.cloneAndAppend({"/jointset/mtp_l/mtp_angle_l/speed", 0});
//     track.set_states_weight_set(statesWeightSet);

//     // Get the underlying MocoStudy.
//     MocoStudy study = track.initialize();
//     MocoProblem& problem = study.updProblem();

//     // Update the weight in the MocoControlGoal that is added to every MocoTrack
//     // problem by default.
//     problem.updGoal("control_effort").setWeight(0.1);

//     // Add a MocoContactTrackingGoal to the problem to track the ground reaction
//     // forces.
//     auto* contactTracking = problem.addGoal<MocoContactTrackingGoal>(
//             "grf_tracking", 1e-2);
//     contactTracking->setExternalLoadsFile("grf_walk.xml");
//     MocoContactTrackingGoalGroup rightContactGroup(contactForcesRight, 
//             "Right_GRF", {"/bodyset/toes_r"});
//     contactTracking->addContactGroup(rightContactGroup);
//     MocoContactTrackingGoalGroup leftContactGroup(contactForcesLeft, 
//             "Left_GRF", {"/bodyset/toes_l"});
//     contactTracking->addContactGroup(leftContactGroup);  

//     // Update the solver tolerances.
//     auto& solver = study.updSolver<MocoCasADiSolver>();
//     solver.set_transcription_scheme("legendre-gauss-radau-3");
//     solver.set_kinematic_constraint_method("Bordalba2023");
//     solver.set_optim_convergence_tolerance(1e-2);
//     solver.set_optim_constraint_tolerance(1e-4);

//     return study;
// }

// /// Track kinematics and ground reaction forces using a torque-driven model.
// void torqueDrivenTracking() {

//     // Construct a torque-driven model.
//     ModelProcessor modelProcessor(loadAndUpdateModel());
//     modelProcessor.append(ModOpRemoveMuscles());
//     modelProcessor.append(ModOpAddReserves(250.0, SimTK::Infinity, true, true));

//     // Construct the base tracking study.
//     TableProcessor tableProcessor = TableProcessor("coordinates.sto") |
//             TabOpUseAbsoluteStateNames() |
//             TabOpAppendCoupledCoordinateValues() |
//             TabOpAppendCoordinateValueDerivativesAsSpeeds();
//     MocoStudy study = constructContactTrackingStudy("torque_driven_tracking", 
//             modelProcessor, tableProcessor);

//     // Update the problem to constrain the initial states to be close to the
//     // reference kinematics at the initial time point.
//     MocoProblem& problem = study.updProblem();
//     Model model = modelProcessor.process();
//     model.initSystem();
//     TimeSeriesTable coordinates = tableProcessor.process(&model);
//     const auto& labels = coordinates.getColumnLabels();
//     for (const auto& label : labels) {
//         if (IO::EndsWith(label, "_beta/value")) { continue; }
//         const auto& value = coordinates.getDependentColumn(label);        
//         double lower = 0.0;
//         double upper = 0.0;
//         if (label.find("/speed") != std::string::npos) {
//             lower = value[0] - 0.1;
//             upper = value[0] + 0.1;
//         } else {
//             lower = value[0] - 0.05;
//             upper = value[0] + 0.05;
//         }

//         problem.setStateInfo(label, {}, {lower, upper});
//     }

//     // Update the solver with the modified MocoProblem.
//     auto& solver = study.updSolver<MocoCasADiSolver>();
//     solver.resetProblem(problem);

//     // Set the guess to the reference kinematics.
//     MocoTrajectory guess = solver.createGuess();
//     guess.insertStatesTrajectory(coordinates);
//     solver.setGuess(guess);

//     model.print("example3DWalking_torque_driven_tracking_model.osim");

//     // Solve!
//     MocoSolution solution = study.solve();
//     solution.write("example3DWalking_torque_driven_tracking_solution.sto");

//     // Extract the ground reaction forces.
//     TimeSeriesTable externalForcesTableFlat = createExternalLoadsTableForGait(
//             model, solution, contactForcesRight, contactForcesLeft);
//     STOFileAdapter::write(externalForcesTableFlat,
//             "example3DWalking_torque_driven_tracking_ground_reactions.sto");
// }

// /// Track kinematics and ground reaction forces using a muscle-driven model, use
// /// the solution from the torque-driven tracking problem as an initial guess, if
// /// available.
// void muscleDrivenTracking() {
    
//     // Construct a muscle-driven model.
//     ModelProcessor modelProcessor(loadAndUpdateModel());
//     // modelProcessor.append(ModOpAddReserves(250.0, SimTK::Infinity, true, true));
//     modelProcessor.append(ModOpIgnoreTendonCompliance());
//     modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
//     modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
//     modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
//     modelProcessor.append(ModOpReplacePathsWithFunctionBasedPaths(
//             "subject_walk_scaled_FunctionBasedPathSet.xml"));

//     // Construct the base tracking study.
//     TimeSeriesTable coordinates("coordinates.sto");
//     coordinates.removeColumn("/jointset/patellofemoral_r/knee_angle_r_beta/value");
//     coordinates.removeColumn("/jointset/patellofemoral_l/knee_angle_l_beta/value");
//     TableProcessor tableProcessor = TableProcessor(coordinates) |
//             TabOpUseAbsoluteStateNames() |
//             TabOpAppendCoupledCoordinateValues() |
//             TabOpAppendCoordinateValueDerivativesAsSpeeds();
//     MocoStudy study = constructContactTrackingStudy("muscle_driven_tracking",
//             modelProcessor, tableProcessor);

//     // Update the weights on the state tracking and contact tracking goals. 
//     auto& problem = study.updProblem();
//     problem.updGoal("state_tracking").setWeight(0.05);
//     problem.updGoal("grf_tracking").setWeight(5e-3);

//     // Constrain the states and controls to be periodic.
//     auto* periodicityGoal = problem.addGoal<MocoPeriodicityGoal>("periodicity");
//     Model model = modelProcessor.process();
//     model.initSystem();
//     for (const auto& coord : model.getComponentList<Coordinate>()) {
//         if (IO::EndsWith(coord.getName(), "_beta")) { continue; }
//         if (!IO::EndsWith(coord.getName(), "_tx")) {
//             periodicityGoal->addStatePair(coord.getStateVariableNames()[0]);
//         }
//         periodicityGoal->addStatePair(coord.getStateVariableNames()[1]);
//     }
//     for (const auto& muscle : model.getComponentList<Muscle>()) {
//         periodicityGoal->addStatePair(muscle.getStateVariableNames()[0]);
//         periodicityGoal->addControlPair(muscle.getAbsolutePathString());
//     }
//     for (const auto& actu : model.getComponentList<Actuator>()) {
//         periodicityGoal->addControlPair(actu.getAbsolutePathString());
//     }

//     // Update the solver with the modified MocoProblem.
//     auto& solver = study.updSolver<MocoCasADiSolver>();
//     solver.resetProblem(problem);

//     // Set the guess to the solution from the torque-driven tracking problem,
//     // if available.
//     if (IO::FileExists("example3DWalking_torque_driven_tracking_solution.sto")) {
//         MocoTrajectory torqueDrivenSolution(
//                 "example3DWalking_torque_driven_tracking_solution.sto");
//         MocoTrajectory guess = solver.createGuess();
//         guess.insertStatesTrajectory(
//                 torqueDrivenSolution.exportToStatesTable(), true);
//         solver.setGuess(guess);
//     }

//     // Solve!
//     MocoSolution solution = study.solve().unseal();
//     solution.write("example3DWalking_muscle_driven_tracking_solution.sto");

//     // Extract the ground reaction forces.
//     TimeSeriesTable externalForcesTableFlat = createExternalLoadsTableForGait(
//             model, solution, contactForcesRight, contactForcesLeft);
//     STOFileAdapter::write(externalForcesTableFlat,
//             "example3DWalking_muscle_driven_tracking_ground_reactions.sto");

//     // Visualize the solution.
//     // study.visualize(solution);
// }

// int main() {

//     // The estimated times below are based on a machine using a 4.7 GHz 
//     // processor with 24 threads.
    
//     // This problem takes ~10 minutes to solve.
//     // torqueDrivenTracking();

//     // This problem takes ~70 minutes to solve, given the initial guess from the
//     // torque-driven tracking problem.
//     muscleDrivenTracking();

//     return EXIT_SUCCESS;
// }

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/VisualizerUtilities.h>
#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>

using namespace OpenSim;

const std::vector<std::string> contactForcesRight = {"/contactHeel_r", 
            "/contactLateralRearfoot_r", "/contactLateralMidfoot_r", 
            "/contactMedialMidfoot_r", "/contactLateralToe_r", 
            "/contactMedialToe_r"};

const std::vector<std::string> contactForcesLeft = {"/contactHeel_l", 
            "/contactLateralRearfoot_l", "/contactLateralMidfoot_l", 
            "/contactMedialMidfoot_l", "/contactLateralToe_l", 
            "/contactMedialToe_l"};

MocoStudy constructContactTrackingStudy(const std::string& studyName, 
        ModelProcessor modelProcessor,
        TableProcessor tableProcessor) {

    MocoTrack track;
    track.setName(studyName);
    Model model = modelProcessor.process();
    model.initSystem();
    track.setModel(modelProcessor);

    track.setStatesReference(tableProcessor);
    track.set_states_global_tracking_weight(0.1);
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);

    MocoWeightSet statesWeightSet;
    statesWeightSet.cloneAndAppend({"/jointset/ground_pelvis/pelvis_ty/value", 0.0});
    statesWeightSet.cloneAndAppend({"/jointset/ground_pelvis/pelvis_ty/speed", 0.1});
    statesWeightSet.cloneAndAppend({"/jointset/mtp_r/mtp_angle_r/value", 0.0});
    statesWeightSet.cloneAndAppend({"/jointset/mtp_r/mtp_angle_r/speed", 0.0});
    statesWeightSet.cloneAndAppend({"/jointset/mtp_l/mtp_angle_l/value", 0.0});
    statesWeightSet.cloneAndAppend({"/jointset/mtp_l/mtp_angle_l/speed", 0.0});
    track.set_states_weight_set(statesWeightSet);

    track.set_initial_time(0.48);
    track.set_final_time(1.61);
    track.set_mesh_interval(0.02);

    MocoStudy study = track.initialize();
    MocoProblem& problem = study.updProblem();

    // Get a reference to the MocoControlGoal that is added to every MocoTrack
    // problem by default.
    MocoControlGoal& effort =
        dynamic_cast<MocoControlGoal&>(problem.updGoal("control_effort"));
    effort.setWeight(0.1);

    // Add a MocoContactTrackingGoal to the problem to track the ground reaction
    // forces.
    auto* contactTracking = problem.addGoal<MocoContactTrackingGoal>(
            "grf_tracking", 1e-2);
    contactTracking->setExternalLoadsFile("grf_walk.xml");
    MocoContactTrackingGoalGroup rightContactGroup(contactForcesRight, 
            "Right_GRF", {"/bodyset/toes_r"});
    contactTracking->addContactGroup(rightContactGroup);
    MocoContactTrackingGoalGroup leftContactGroup(contactForcesLeft, 
            "Left_GRF", {"/bodyset/toes_l"});
    contactTracking->addContactGroup(leftContactGroup);  

    // Update the solver tolerances.
    auto& solver = study.updSolver<MocoCasADiSolver>();
    solver.set_transcription_scheme("legendre-gauss-radau-3");
    solver.set_kinematic_constraint_method("Bordalba2023");
    solver.set_optim_convergence_tolerance(1e-2);
    solver.set_optim_constraint_tolerance(1e-4);

    return study;
}


void createInitialGuess(Model model) {

    ModelProcessor modelProcessor(model);
    modelProcessor.append(ModOpRemoveMuscles());
    modelProcessor.append(ModOpAddReserves(250.0, SimTK::Infinity, true, true));

    TimeSeriesTable coordinates("coordinates.sto");
    coordinates.removeColumn("/jointset/patellofemoral_r/knee_angle_r_beta/value");
    coordinates.removeColumn("/jointset/patellofemoral_l/knee_angle_l_beta/value");
    TableProcessor tableProcessor = 
            TableProcessor(coordinates) |
            TabOpUseAbsoluteStateNames() |
            TabOpAppendCoupledCoordinateValues() |
            TabOpAppendCoordinateValueDerivativesAsSpeeds();

    MocoStudy study = constructContactTrackingStudy("create_initial_guess", 
            modelProcessor, tableProcessor);

    MocoProblem& problem = study.updProblem();

    Model modelUpdated = modelProcessor.process();
    modelUpdated.initSystem();

    // Constrain the initial states to be close to the reference.
    TimeSeriesTable coordinatesUpdated = tableProcessor.process(&modelUpdated);
    const auto& labels = coordinatesUpdated.getColumnLabels();
    for (const auto& label : labels) {
        const auto& value = coordinatesUpdated.getDependentColumn(label);        
        double lower = 0.0;
        double upper = 0.0;
        if (label.find("/speed") != std::string::npos) {
            lower = value[0] - 0.1;
            upper = value[0] + 0.1;
        } else {
            lower = value[0] - 0.05;
            upper = value[0] + 0.05;
        }

        problem.setStateInfo(label, {}, {lower, upper});
    }

    // Update the solver tolerances.
    auto& solver = study.updSolver<MocoCasADiSolver>();
    solver.resetProblem(problem);

    Model modelSolution = modelProcessor.process();
    modelSolution.initSystem();
    TimeSeriesTable initialStates = tableProcessor.process(&modelSolution);
    MocoTrajectory guess = solver.createGuess();
    guess.insertStatesTrajectory(initialStates);
    solver.setGuess(guess);

    // Solve!
    MocoSolution solution = study.solve().unseal();
    solution.write("example3DWalking_initial_guess.sto");

    // Print the model.
    modelSolution.print("example3DWalking_initial_guess_model.osim");

    // Extract the ground reaction forces.
    TimeSeriesTable externalForcesTableFlat = createExternalLoadsTableForGait(
            modelSolution, solution, contactForcesRight, contactForcesLeft);
    STOFileAdapter::write(externalForcesTableFlat,
            "example3DWalking_initial_guess_ground_reactions.sto");

}

void trackWalking(Model model) {

    MocoTrack track;
    track.setName("track_walking");
    
    ModelProcessor modelProcessor(model);
    modelProcessor.append(ModOpIgnoreTendonCompliance());
    modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
    modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
    modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
    modelProcessor.append(ModOpReplacePathsWithFunctionBasedPaths(
            "subject_walk_scaled_FunctionBasedPathSet.xml"));

    TimeSeriesTable coordinates("coordinates.sto");
    coordinates.removeColumn("/jointset/patellofemoral_r/knee_angle_r_beta/value");
    coordinates.removeColumn("/jointset/patellofemoral_l/knee_angle_l_beta/value");
    TableProcessor tableProcessor = TableProcessor(coordinates) |
            TabOpUseAbsoluteStateNames() |
            TabOpAppendCoupledCoordinateValues() |
            TabOpAppendCoordinateValueDerivativesAsSpeeds();
    
    MocoStudy study = constructContactTrackingStudy("track_walking",
            modelProcessor, tableProcessor);

    auto& problem = study.updProblem();
    problem.updGoal("state_tracking").setWeight(0.05);
    problem.updGoal("grf_tracking").setWeight(5e-3);
    // problem.addGoal<MocoInitialActivationGoal>("initial_activation");

    // Constrain the states and controls to be periodic.
    auto* periodicityGoal = problem.addGoal<MocoPeriodicityGoal>("periodicity");
    model.initSystem();
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


    auto& solver = study.updSolver<MocoCasADiSolver>();
    solver.resetProblem(problem);

    MocoTrajectory initialGuessSolution("example3DWalking_initial_guess.sto");
    MocoTrajectory guess = solver.createGuess();
    guess.insertStatesTrajectory(initialGuessSolution.exportToStatesTable());
    solver.setGuess(guess);

    // Solve!
    MocoSolution solution = study.solve().unseal();
    solution.write("example3DWalking_track_walking.sto");
    // MocoTrajectory solution("example3DWalking_track_walking.sto");
    // study.visualize(solution);

    // Print the model.
    // Model modelSolution = modelProcessor.process();
    // modelSolution.initSystem();
    // modelSolution.print("example3DWalking_track_walking_model.osim");

    // // Extract the ground reaction forces.
    // TimeSeriesTable externalForcesTableFlat = createExternalLoadsTableForGait(
    //         modelSolution, solution, contactForcesRight, contactForcesLeft);
    // STOFileAdapter::write(externalForcesTableFlat,
    //         "example3DWalking_track_walking_ground_reactions.sto");

    // Visualize the solution.
    // study.visualize(solution);
}


int main() {

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

    // createInitialGuess(model);

    // createPeriodicInitialGuess(model);

    trackWalking(model);


    return EXIT_SUCCESS;
}