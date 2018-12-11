/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: sandboxOneLegCouplerKnee.cpp                             *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include <Muscollo/osimMuscollo.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Tools/IKTaskSet.h>
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>

using namespace OpenSim;
using SimTK::Vec3;
using SimTK::Inertia;

/// Convenience function to apply an CoordinateActuator to the model.
void addCoordinateActuator(Model& model, std::string coordName,
    double optimalForce) {

    auto& coordSet = model.updCoordinateSet();

    auto* actu = new CoordinateActuator();
    actu->setName("tau_" + coordName);
    actu->setCoordinate(&coordSet.get(coordName));
    actu->setOptimalForce(optimalForce);
    actu->setMinControl(-1);
    actu->setMaxControl(1);
    model.addComponent(actu);
}

Model createRightLegWeldedPelvisModel(const std::string& actuatorType) {
    Model model("Rajagopal2015_right_leg_9musc.osim");
    model.finalizeConnections(); // need this here to access offset frames

    // Remove ground_pelvis joint and replace with weld
    auto& ground_pelvis = model.updJointSet().get("ground_pelvis");
    model.updJointSet().remove(&ground_pelvis);
    const auto& pelvis = model.getBodySet().get("pelvis");
    auto* ground_pelvis_weld = new WeldJoint("ground_pelvis", model.getGround(),
        Vec3(0, 1.2, 0), Vec3(0), pelvis, Vec3(0), Vec3(0));
    model.addJoint(ground_pelvis_weld);

    // Remove subtalar joint and replace with weld
    auto& subtalar_r = model.updJointSet().get("subtalar_r");
    PhysicalOffsetFrame* talus_r_offset_subtalar_r
        = PhysicalOffsetFrame().safeDownCast(subtalar_r.getParentFrame().clone());
    PhysicalOffsetFrame* calcn_r_offset_subtalar_r
        = PhysicalOffsetFrame().safeDownCast(subtalar_r.getChildFrame().clone());
    model.updJointSet().remove(&subtalar_r);
    auto* subtalar_r_weld = new WeldJoint("subtalar_r",
        model.getBodySet().get("talus_r"),
        talus_r_offset_subtalar_r->get_translation(),
        talus_r_offset_subtalar_r->get_orientation(),
        model.getBodySet().get("calcn_r"),
        calcn_r_offset_subtalar_r->get_translation(),
        calcn_r_offset_subtalar_r->get_orientation());
    model.addJoint(subtalar_r_weld);

    // Remove mtp joint and replace with weld
    auto& mtp_r = model.updJointSet().get("mtp_r");
    PhysicalOffsetFrame* calcn_r_offset_mtp_r
        = PhysicalOffsetFrame().safeDownCast(mtp_r.getParentFrame().clone());
    PhysicalOffsetFrame* toes_r_offset_mtp_r
        = PhysicalOffsetFrame().safeDownCast(mtp_r.getChildFrame().clone());
    model.updJointSet().remove(&mtp_r);
    auto* mtp_r_weld = new WeldJoint("mtp_r",
        model.getBodySet().get("calcn_r"),
        calcn_r_offset_mtp_r->get_translation(),
        calcn_r_offset_mtp_r->get_orientation(),
        model.getBodySet().get("toes_r"),
        toes_r_offset_mtp_r->get_translation(),
        toes_r_offset_mtp_r->get_orientation());
    model.addJoint(mtp_r_weld);


    if (actuatorType == "torques") {
        // Remove muscles and add coordinate actuators
        addCoordinateActuator(model, "hip_flexion_r", 20);
        //addCoordinateActuator(model, "hip_adduction_r", 20);
        //addCoordinateActuator(model, "hip_rotation_r", 20);
        addCoordinateActuator(model, "knee_angle_r", 20);
        addCoordinateActuator(model, "knee_angle_r_beta", 20);
        addCoordinateActuator(model, "ankle_angle_r", 5);
        removeMuscles(model);
    } else if (actuatorType == "path_actuators") {
        replaceMusclesWithPathActuators(model);
    } else if (actuatorType == "muscles") {
        addCoordinateActuator(model, "hip_flexion_r", 20);
        addCoordinateActuator(model, "hip_adduction_r", 20);
        addCoordinateActuator(model, "hip_rotation_r", 20);
        addCoordinateActuator(model, "knee_angle_r", 20);
        addCoordinateActuator(model, "knee_angle_r_beta", 20);
        addCoordinateActuator(model, "ankle_angle_r", 5);
    } else {
        OPENSIM_THROW(Exception, "Invalid actuator type");
    }

    // Finalize model and print.
    model.finalizeFromProperties();
    model.finalizeConnections();
    model.print("RightLegWeldedPelvis_" + actuatorType + ".osim");

    return model;
}

Model createRightLegModel(const std::string& actuatorType) {
    Model model("Rajagopal2015_right_leg_9musc.osim");
    model.finalizeConnections(); // need this here to access offset frames

    // Remove subtalar joint and replace with weld
    auto& subtalar_r = model.updJointSet().get("subtalar_r");
    PhysicalOffsetFrame* talus_r_offset_subtalar_r
        = PhysicalOffsetFrame().safeDownCast(subtalar_r.getParentFrame().clone());
    PhysicalOffsetFrame* calcn_r_offset_subtalar_r
        = PhysicalOffsetFrame().safeDownCast(subtalar_r.getChildFrame().clone());
    model.updJointSet().remove(&subtalar_r);
    auto* subtalar_r_weld = new WeldJoint("subtalar_r",
        model.getBodySet().get("talus_r"),
        talus_r_offset_subtalar_r->get_translation(),
        talus_r_offset_subtalar_r->get_orientation(),
        model.getBodySet().get("calcn_r"),
        calcn_r_offset_subtalar_r->get_translation(),
        calcn_r_offset_subtalar_r->get_orientation());
    model.addJoint(subtalar_r_weld);

    // Remove mtp joint and replace with weld
    auto& mtp_r = model.updJointSet().get("mtp_r");
    PhysicalOffsetFrame* calcn_r_offset_mtp_r
        = PhysicalOffsetFrame().safeDownCast(mtp_r.getParentFrame().clone());
    PhysicalOffsetFrame* toes_r_offset_mtp_r
        = PhysicalOffsetFrame().safeDownCast(mtp_r.getChildFrame().clone());
    model.updJointSet().remove(&mtp_r);
    auto* mtp_r_weld = new WeldJoint("mtp_r",
        model.getBodySet().get("calcn_r"),
        calcn_r_offset_mtp_r->get_translation(),
        calcn_r_offset_mtp_r->get_orientation(),
        model.getBodySet().get("toes_r"),
        toes_r_offset_mtp_r->get_translation(),
        toes_r_offset_mtp_r->get_orientation());
    model.addJoint(mtp_r_weld);


    if (actuatorType == "torques") {
        // Remove muscles and add coordinate actuators
        addCoordinateActuator(model, "pelvis_tx", 500);
        addCoordinateActuator(model, "pelvis_ty", 500);
        addCoordinateActuator(model, "pelvis_tz", 500);
        addCoordinateActuator(model, "pelvis_tilt", 25);
        addCoordinateActuator(model, "pelvis_list", 25);
        addCoordinateActuator(model, "pelvis_rotation", 25);
        addCoordinateActuator(model, "hip_angle_r", 20);
        addCoordinateActuator(model, "hip_adduction_r", 20);
        addCoordinateActuator(model, "hip_rotation_r", 20);
        addCoordinateActuator(model, "knee_angle_r", 20);
        addCoordinateActuator(model, "knee_angle_r_beta", 20);
        addCoordinateActuator(model, "ankle_angle_r", 5);
        removeMuscles(model);
    }
    else {
        OPENSIM_THROW(Exception, "Invalid actuator type");
    }

    // Finalize model and print.
    model.finalizeFromProperties();
    model.finalizeConnections();
    model.print("RightLeg_" + actuatorType + ".osim");

    return model;
}

void minimizeControlEffortRightLegWeldedPelvis(const std::string& actuatorType) 
{
    MucoTool muco;
    muco.setName("sandboxRightLeg_weldedPelvis_" + actuatorType +
        "_minimize_control_effort");
    MucoProblem& mp = muco.updProblem();
    mp.setModelCopy(createRightLegWeldedPelvisModel(actuatorType));

    // Set bounds.
    mp.setTimeBounds(0, 1);
    mp.setStateInfo("/jointset/hip_r/hip_flexion_r/value", {-10, 10},
        -SimTK::Pi / 4.0, SimTK::Pi / 4.0);
    mp.setStateInfo("/jointset/hip_r/hip_rotation_r/value", {-10, 10});
    mp.setStateInfo("/jointset/hip_r/hip_adduction_r/value", {-10, 10});
    mp.setStateInfo("/jointset/walker_knee_r/knee_angle_r/value", {-10, 10}, 
        -0.6, 0);
    mp.setStateInfo("/jointset/patellofemoral_r/knee_angle_r_beta/value", 
        {-10, 10});
    mp.setStateInfo("/jointset/ankle_r/ankle_angle_r/value", {-10, 10}, -0.1,
        0.1);

    mp.setControlInfo("/forceset/bifemsh_r", {0.01, 1});
    mp.setControlInfo("/forceset/med_gas_r", {0.01, 1});
    mp.setControlInfo("/forceset/glut_max2_r", {0.01, 1});
    mp.setControlInfo("/forceset/psoas_r", {0.01, 1.5});
    mp.setControlInfo("/forceset/rect_fem_r", {0.01, 1});
    mp.setControlInfo("/forceset/semimem_r", {0.01, 1});
    mp.setControlInfo("/forceset/soleus_r", {0.01, 1});
    mp.setControlInfo("/forceset/tib_ant_r", {0.01, 1});
    mp.setControlInfo("/forceset/vas_int_r", {0.01, 1});

    auto* effort = mp.addCost<MucoControlCost>();
    effort->setName("control_effort");
    effort->setWeight("tau_hip_flexion_r", 1000);
    effort->setWeight("tau_hip_adduction_r", 1000);
    effort->setWeight("tau_hip_rotation_r", 1000);
    effort->setWeight("tau_knee_angle_r", 1000);
    effort->setWeight("tau_knee_angle_r_beta", 1000);
    effort->setWeight("tau_ankle_angle_r", 1000);

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(10);
    ms.set_verbosity(2);
    ms.set_optim_solver("snopt");
    ms.set_optim_convergence_tolerance(1e-3);
    ms.set_optim_hessian_approximation("exact");
    ms.set_hessian_block_sparsity_mode("dense");
    ms.set_transcription_scheme("hermite-simpson");
    ms.set_enforce_constraint_derivatives(true);
    ms.set_lagrange_multiplier_weight(10);
    ms.set_velocity_correction_bounds({-0.25, 0.25});
    //ms.setGuess("bounds");
    ms.setGuessFile("sandboxRightLeg_weldedPelvis_" +
        actuatorType + "_minimize_control_effort_solution.sto");

    MucoSolution solution = muco.solve().unseal();
    muco.visualize(solution);
}

void stateTrackingRightLegWeldedPelvis(const std::string& actuatorType) {
    MucoTool muco;
    muco.setName("sandboxRightLeg_weldedPelvis_" + actuatorType +
        "_state_tracking");
    MucoProblem& mp = muco.updProblem();
    Model model = createRightLegWeldedPelvisModel(actuatorType);
    mp.setModelCopy(model);

    auto markersRef = TRCFileAdapter::read("marker_trajectories.trc");
    auto time = markersRef.getIndependentColumn();
    mp.setTimeBounds(0, time.back());

    auto* stateTracking = mp.addCost<MucoStateTrackingCost>();
    stateTracking->setName("state_tracking");
    auto statesRef = STOFileAdapter::read("ik_results.sto");
    stateTracking->setReference(statesRef);
    stateTracking->setWeight("/jointset/knee_r/knee_angle_r_beta/value", 0);
    stateTracking->setAllowUnusedReferences(true);
    

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(50);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-3);
    ms.set_optim_hessian_approximation("exact");
    ms.set_hessian_block_sparsity_mode("dense");
    ms.set_transcription_scheme("hermite-simpson");
    ms.set_enforce_constraint_derivatives(true);
    ms.set_lagrange_multiplier_weight(10);

    MucoIterate guess = ms.createGuess();
    model.initSystem();
    model.getSimbodyEngine().convertDegreesToRadians(statesRef);
    STOFileAdapter::write(statesRef, "ik_results_radians.sto");
    guess.setStatesTrajectory(statesRef, true, true);
    ms.setGuess(guess);

    MucoSolution solution = muco.solve();
    muco.visualize(solution);
}

void markerTrackingRightLeg(const std::string& actuatorType) {
    MucoTool muco;
    muco.setName("sandboxRightLeg_" + actuatorType +
        "_marker_tracking");
    MucoProblem& mp = muco.updProblem();
    Model model = createRightLegModel(actuatorType);
    mp.setModelCopy(model);

    auto markersRef = TRCFileAdapter::read("marker_tracking.trc");
    auto time = markersRef.getIndependentColumn();
    mp.setTimeBounds(0, time.back());

    auto* markerTracking = mp.addCost<MucoMarkerTrackingCost>();
    markerTracking->setName("marker_tracking");
    InverseKinematicsTool iktool("ik_setup.xml");
    IKTaskSet& tasks = iktool.getIKTaskSet();
    Set<MarkerWeight> markerWeights;
    tasks.createMarkerWeightSet(markerWeights);
    markerTracking->setMarkersReference(MarkersReference(markersRef,
        &markerWeights));
    markerTracking->setAllowUnusedReferences(true);

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(10);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_convergence_tolerance(1e-3);
    ms.set_optim_hessian_approximation("exact");
    ms.set_hessian_block_sparsity_mode("dense");
    ms.set_transcription_scheme("hermite-simpson");
    ms.set_enforce_constraint_derivatives(true);
    ms.set_lagrange_multiplier_weight(10);

    MucoIterate guess = ms.createGuess();
    guess.setStatesTrajectory(
        STOFileAdapter::read("ik_results_radians.sto"), true, true);
    ms.setGuess(guess);

    MucoSolution solution = muco.solve();
    muco.visualize(solution);
}

void main() {

    // This problem solves fine in SNOPT when using coordinate actuators.
    // However, for path actuators and muscles, SNOPT exits with error 52:
    // "incorrect constraint derivatives". This may suggest a bug in our own
    // Jacobian derivative calculations. But why only for path actuators and
    // muscles?
    minimizeControlEffortRightLegWeldedPelvis("muscles");
    //stateTrackingWeldedLeg39("torques");
    //markerTrackingRightLeg("torques");
}