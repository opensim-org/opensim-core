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
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/VisualizerUtilities.h>
#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Actuators/CoordinateActuator.h>

using namespace OpenSim;

void addContactsToModel(Model& model) {
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
}

void addGroundToFootJoint(Model& model, const std::string& footBodyName) {
    SpatialTransform transform;
    transform[0].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_rx", footBodyName), 1, 1));
    transform[0].setFunction(new LinearFunction());
    transform[0].setAxis(SimTK::Vec3(0, 0, 1));

    transform[1].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_ry", footBodyName), 1, 1));
    transform[1].setFunction(new LinearFunction());
    transform[1].setAxis(SimTK::Vec3(1, 0, 0));

    transform[2].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_rz", footBodyName), 1, 1));
    transform[2].setFunction(new LinearFunction());
    transform[2].setAxis(SimTK::Vec3(0, 1, 0));

    transform[3].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_tx", footBodyName), 1, 1));
    transform[3].setFunction(new LinearFunction());
    transform[3].setAxis(SimTK::Vec3(1, 0, 0));

    transform[4].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_ty", footBodyName), 1, 1));
    transform[4].setFunction(new LinearFunction());
    transform[4].setAxis(SimTK::Vec3(0, 1, 0));

    transform[5].setCoordinateNames(OpenSim::Array<std::string>(
            fmt::format("{}_tz", footBodyName), 1, 1));
    transform[5].setFunction(new LinearFunction());
    transform[5].setAxis(SimTK::Vec3(0, 0, 1));

    CustomJoint* joint = new CustomJoint(
            fmt::format("ground_{}", footBodyName), 
            model.getGround(), 
            model.getComponent<Body>(fmt::format("/bodyset/{}", footBodyName)), 
            transform);
    model.addJoint(joint);
}

void addToeStiffnessAndDamping(Model& model) {
    ExpressionBasedCoordinateForce* ebcf_toes_l = 
        new ExpressionBasedCoordinateForce("mtp_angle_l", "-25.0*q-2.0*qdot");
    model.addComponent(ebcf_toes_l);
    ExpressionBasedCoordinateForce* ebcf_toes_r = 
        new ExpressionBasedCoordinateForce("mtp_angle_r", "-25.0*q-2.0*qdot");
    model.addComponent(ebcf_toes_r);
}

void addCoordinateActuator(Model& model, const std::string& coordinateName, 
        double optimalForce) {
    CoordinateActuator* actuator = new CoordinateActuator(coordinateName);
    actuator->setName(fmt::format("{}_actuator", coordinateName));
    actuator->setOptimalForce(optimalForce);
    actuator->setMinControl(-1.0);
    actuator->setMaxControl(1.0);
    model.addComponent(actuator);
}

int main() {

    // Load the base model.
    Model model("subject_walk_scaled.osim");
    model.initSystem();

    // Create a model with only the feet of the original model.
    Model feetModel;
    feetModel.addBody(model.getComponent<Body>("/bodyset/calcn_r").clone());
    feetModel.addBody(model.getComponent<Body>("/bodyset/toes_r").clone());
    feetModel.addJoint(model.getComponent<Joint>("/jointset/mtp_r").clone());
    feetModel.addBody(model.getComponent<Body>("/bodyset/calcn_l").clone());
    feetModel.addBody(model.getComponent<Body>("/bodyset/toes_l").clone());
    feetModel.addJoint(model.getComponent<Joint>("/jointset/mtp_l").clone());

    // Define the ground-to-foot joints using CustomJoints.
    addGroundToFootJoint(feetModel, "calcn_r");
    addGroundToFootJoint(feetModel, "calcn_l");

    // Add the markers from the feet to the new model.
    std::vector<std::string> feetMarkers = {"Toe", "ToeGround", "MT5", 
            "MT5Ground", "Heel", "HeelGround"};
    for (const auto& markerName : feetMarkers) {
        for (const auto& side : {"R.", "L."}) {
            auto marker = model.getComponent<Marker>(
                    fmt::format("/markerset/{}{}", side, markerName)).clone();
            feetModel.addMarker(marker);
        }
    }
    feetModel.finalizeConnections();

    // Add the contact geometry and forces to the feet model.
    addContactsToModel(feetModel);

    // Add toe stiffness and damping to the feet model.
    addToeStiffnessAndDamping(feetModel);

    // Add strong actuators to the model.
    for (const auto& side : {"l", "r"}) {
        addCoordinateActuator(feetModel, 
                fmt::format("calcn_{}_tx", side), 2000);
        addCoordinateActuator(feetModel, 
                fmt::format("calcn_{}_ty", side), 3000);
        addCoordinateActuator(feetModel, 
                fmt::format("calcn_{}_tx", side), 2000);
        for (const auto& axis : {"x", "y", "z"}) {
            addCoordinateActuator(feetModel, 
                fmt::format("calcn_{}_r{}", side, axis), 1000);
        }
        addCoordinateActuator(feetModel, fmt::format("mtp_angle_{}", side), 250);
    }

    // Print the model to a file.
    feetModel.print("feet.osim");

    // Run inverse kinematics on the foot model.
    Set<MarkerWeight> markerWeights;
    markerWeights.cloneAndAppend({"R.Heel", 10});
    markerWeights.cloneAndAppend({"R.MT5", 5});
    markerWeights.cloneAndAppend({"R.Toe", 2});
    markerWeights.cloneAndAppend({"L.Heel", 10});
    markerWeights.cloneAndAppend({"L.MT5", 5});
    markerWeights.cloneAndAppend({"L.Toe", 2});
    markerWeights.cloneAndAppend({"R.HeelGround", 1});
    markerWeights.cloneAndAppend({"R.MT5Ground", 1});
    markerWeights.cloneAndAppend({"R.ToeGround", 1});
    markerWeights.cloneAndAppend({"L.HeelGround", 1});
    markerWeights.cloneAndAppend({"L.MT5Ground", 1});
    markerWeights.cloneAndAppend({"L.ToeGround", 1});

    IKTaskSet ikTaskSet;
    ikTaskSet.createMarkerWeightSet(markerWeights);

    InverseKinematicsTool iktool;
    iktool.setModel(feetModel);
    iktool.setMarkerDataFileName("marker_trajectories.trc");
    iktool.set_IKTaskSet(ikTaskSet);
    iktool.setOutputMotionFileName("feet_ik_solution.sto");    
    iktool.run();

    // Update the IK solution to something that we can visualize and use 
    // in Moco.
    TableProcessor tableProcessor = 
            TableProcessor("feet_ik_solution.sto") |
            TabOpConvertDegreesToRadians() |
            TabOpAppendCoordinateValueDerivativesAsSpeeds() |
            TabOpUseAbsoluteStateNames();
    TimeSeriesTable feetCoordinateReference = 
            tableProcessor.processAndConvertToRadians("", feetModel);
    STOFileAdapter::write(feetCoordinateReference, 
            "feet_coordinate_reference.sto");

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
    MocoSolution solution = study.solve().unseal();
    solution.write("feet_tracking_solution.sto");

    // Extract the ground reaction forces.
    std::vector<std::string> contact_r = {"contactHeel_r", 
            "contactLateralRearfoot_r", "contactLateralMidfoot_r", 
            "contactMedialMidfoot_r", "contactLateralToe_r", 
            "contactMedialToe_r"};
    std::vector<std::string> contact_l = {"contactHeel_l", 
            "contactLateralRearfoot_l", "contactLateralMidfoot_l", 
            "contactMedialMidfoot_l", "contactLateralToe_l", 
            "contactMedialToe_l"};
    TimeSeriesTable externalForcesTableFlat = createExternalLoadsTableForGait(
            feetModel, solution, contact_r, contact_l);
    STOFileAdapter::write(externalForcesTableFlat,
            "feet_tracking_solution_ground_reactions.sto");

    study.visualize((solution));

    return EXIT_SUCCESS;
}