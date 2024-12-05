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

/// This example demonstrates how to use the MocoTrack tool to solve a 3D 
/// walking optimization that tracks kinematics and ground reaction forces using 
/// a foot-ground contact model. This example uses a sequential approach to 
/// achieve the desired motion:
///  - First, we update the model by attaching foot-ground contact elements and
///    adding forces to the toe joints.
///  - Second, we create an initial guess for the skeletal kinematics by solving
///    a torque-driven tracking problem. 
///  - Third, we solve a muscle-driven tracking problem that again tracks the 
///    experimental kinematics and ground reaction forces while also enforcing
///    periodicity constraints on the states and controls.
/// 
/// See the README.txt next to this file for more information.

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

    createInitialGuess(model);

    trackWalking(model);


    return EXIT_SUCCESS;
}