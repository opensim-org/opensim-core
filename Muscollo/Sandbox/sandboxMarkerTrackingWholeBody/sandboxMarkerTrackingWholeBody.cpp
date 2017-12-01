/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: exampleMarkerTrackingWholeBody.cpp                       *
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

 /// Solves two tracking problem using a 10 DOF OpenSim model.

#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <Muscollo/osimMuscollo.h>

using namespace OpenSim;

/// Similar to CoordinateActuator (simply produces a generalized force) but
/// with first-order linear activation dynamics. This actuator has one state
/// variable, `activation`, with \f$ \dot{a} = (u - a) / \tau \f$, where
/// \f$ a \f$ is activation, \f$ u $\f is excitation, and \f$ \tau \f$ is the
/// activation time constant (there is no separate deactivation time constant).
/// <b>Default %Property Values</b>
/// @verbatim
/// activation_time_constant: 0.01
/// default_activation: 0.5
/// @dverbatim
class /*OSIMMUSCOLLO_API*/
    ActivationCoordinateActuator : public CoordinateActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ActivationCoordinateActuator,
        CoordinateActuator);
public:
    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
        "Larger value means activation can change more rapidly (units: seconds).");

    OpenSim_DECLARE_PROPERTY(default_activation, double,
        "Value of activation in the default state returned by initSystem().");

    ActivationCoordinateActuator() {
        constructProperties();
    }

    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        Super::extendAddToSystem(system);
        addStateVariable("activation", SimTK::Stage::Dynamics);
    }

    void extendInitStateFromProperties(SimTK::State& s) const override {
        Super::extendInitStateFromProperties(s);
        setStateVariableValue(s, "activation", get_default_activation());
    }

    void extendSetPropertiesFromState(const SimTK::State& s) override {
        Super::extendSetPropertiesFromState(s);
        set_default_activation(getStateVariableValue(s, "activation"));
    }

    // TODO no need to do clamping, etc; CoordinateActuator is bidirectional.
    void computeStateVariableDerivatives(const SimTK::State& s) const override {
        const auto& tau = get_activation_time_constant();
        const auto& u = getControl(s);
        const auto& a = getStateVariableValue(s, "activation");
        const SimTK::Real adot = (u - a) / tau;
        setStateVariableDerivativeValue(s, "activation", adot);
    }

    double computeActuation(const SimTK::State& s) const override {
        return getStateVariableValue(s, "activation") * getOptimalForce();
    }
private:
    void constructProperties() {
        constructProperty_activation_time_constant(0.010);
        constructProperty_default_activation(0.5);
    }
};

/// Convenience function for apply an ActivationCoordinateActuator to a model.
void addActivationCoordinateActuator(Model& model, std::string coordName,
    double optimalForce) {

    auto& coordSet = model.updCoordinateSet();

    auto* actu = new ActivationCoordinateActuator();
    actu->set_default_activation(0.1);
    actu->setName("tau_" + coordName);
    actu->setCoordinate(&coordSet.get(coordName));
    actu->setOptimalForce(optimalForce);
    model.addComponent(actu);
}

/// Set the model and bounds for the specified MucoProblem.
void setModelAndBounds(MucoProblem& mp) {

    Model model("subject01.osim");

    addActivationCoordinateActuator(model, "lumbar_extension", 100);
    addActivationCoordinateActuator(model, "pelvis_tilt", 100);
    addActivationCoordinateActuator(model, "pelvis_tx", 1000);
    addActivationCoordinateActuator(model, "pelvis_ty", 1000);
    addActivationCoordinateActuator(model, "hip_flexion_r", 100);
    addActivationCoordinateActuator(model, "knee_angle_r", 100);
    addActivationCoordinateActuator(model, "ankle_angle_r", 100);
    addActivationCoordinateActuator(model, "hip_flexion_l", 100);
    addActivationCoordinateActuator(model, "knee_angle_l", 100);
    addActivationCoordinateActuator(model, "ankle_angle_l", 100);

    // Model(dynamics).
    // -----------------
    mp.setModel(model);

    // Bounds.
    // -------
    double finalTime = 1.25;
    mp.setTimeBounds(0, finalTime);
    mp.setStateInfo("ground_pelvis/pelvis_tilt/value", { -10, 10 });
    mp.setStateInfo("ground_pelvis/pelvis_tilt/speed", { -50, 50 });
    mp.setStateInfo("ground_pelvis/pelvis_tx/value", { -10, 10 });
    mp.setStateInfo("ground_pelvis/pelvis_tx/speed", { -50, 50 });
    mp.setStateInfo("ground_pelvis/pelvis_ty/value", { -10, 10 });
    mp.setStateInfo("ground_pelvis/pelvis_ty/speed", { -50, 50 });
    mp.setStateInfo("hip_r/hip_flexion_r/value", { -10, 10 });
    mp.setStateInfo("hip_r/hip_flexion_r/speed", { -50, 50 });
    mp.setStateInfo("knee_r/knee_angle_r/value", { -10, 10 });
    mp.setStateInfo("knee_r/knee_angle_r/speed", { -50, 50 });
    mp.setStateInfo("ankle_r/ankle_angle_r/value", { -10, 10 });
    mp.setStateInfo("ankle_r/ankle_angle_r/speed", { -50, 50 });
    mp.setStateInfo("hip_l/hip_flexion_l/value", { -10, 10 });
    mp.setStateInfo("hip_l/hip_flexion_l/speed", { -50, 50 });
    mp.setStateInfo("knee_l/knee_angle_l/value", { -10, 10 });
    mp.setStateInfo("knee_l/knee_angle_l/speed", { -50, 50 });
    mp.setStateInfo("ankle_l/ankle_angle_l/value", { -10, 10 });
    mp.setStateInfo("ankle_l/ankle_angle_l/speed", { -50, 50 });
    mp.setStateInfo("back/lumbar_extension/value", { -10, 10 });
    mp.setStateInfo("back/lumbar_extension/speed", { -50, 50 });
    mp.setStateInfo("tau_lumbar_extension/activation", { -1, 1 });
    mp.setStateInfo("tau_pelvis_tilt/activation", {-1, 1});
    mp.setStateInfo("tau_pelvis_tx/activation", { -1, 1 });
    mp.setStateInfo("tau_pelvis_ty/activation", { -1, 1 });
    mp.setStateInfo("tau_hip_flexion_r/activation", { -1, 1 });
    mp.setStateInfo("tau_knee_angle_r/activation", { -1, 1 });
    mp.setStateInfo("tau_ankle_angle_r/activation", { -1, 1 });
    mp.setStateInfo("tau_hip_flexion_l/activation", { -1, 1 });
    mp.setStateInfo("tau_knee_angle_l/activation", { -1, 1 });
    mp.setStateInfo("tau_ankle_angle_l/activation", { -1, 1 });
    mp.setControlInfo("tau_lumbar_extension", { -1, 1 });
    mp.setControlInfo("tau_pelvis_tilt", { -1, 1 });
    mp.setControlInfo("tau_pelvis_tx", { -1, 1 });
    mp.setControlInfo("tau_pelvis_ty", { -1, 1 });
    mp.setControlInfo("tau_hip_flexion_r", { -1, 1 });
    mp.setControlInfo("tau_knee_angle_r", { -1, 1 });
    mp.setControlInfo("tau_ankle_angle_r", { -1, 1 });
    mp.setControlInfo("tau_hip_flexion_l", { -1, 1 });
    mp.setControlInfo("tau_knee_angle_l", { -1, 1 });
    mp.setControlInfo("tau_ankle_angle_l", { -1, 1 });
}

/// Solve a full-body (10 DOF) tracking problem by having each model 
/// generalized coordinate track the coordinate value obtained from 
/// inverse kinematics.
MucoSolution solveStateTrackingProblem() {

    MucoTool muco;
    muco.setName("whole_body_state_tracking");

    // Define the optimal control problem.
    // ===================================
    MucoProblem& mp = muco.updProblem();

    // Bounds and model.
    setModelAndBounds(mp);

    // Cost.
    // -----
    MucoStateTrackingCost tracking;
    auto ref = STOFileAdapter::read("state_reference.mot");
    auto refFilt = filterLowpass(ref, 6.0, true);
    tracking.setReference(refFilt);
    //tracking.setAllowUnusedReferences(true);
    mp.addCost(tracking);

    // Configure the solver.
    // =====================
    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(50);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_hessian_approximation("exact");

    // Create guess.
    // =============
    MucoIterate guess = ms.createGuess();
    auto model = mp.getPhase().getModel();
    model.initSystem();
    auto refFilt2 = refFilt;
    model.getSimbodyEngine().convertDegreesToRadians(refFilt2);
    STOFileAdapter::write(refFilt2, "state_reference_radians.sto");
    guess.setStatesTrajectory(refFilt2, true, true);
    ms.setGuess(guess);

    // Solve the problem.
    // ==================
    MucoSolution solution = muco.solve();
    solution.write("sandboxMarkerTrackingWholeBody_states_solution.sto");

    muco.visualize(solution);

    return solution;
}

/// Solve a full-body (10 DOF) tracking problem by having the model markers
/// track the marker trajectories directly.
MucoSolution solveMarkerTrackingProblem() {

    MucoTool muco;
    muco.setName("whole_body_marker_tracking");

    // Define the optimal control problem.
    // ===================================
    MucoProblem& mp = muco.updProblem();

    // Bounds and model.
    setModelAndBounds(mp);
        
    // Cost.
    // -----
    MucoMarkerTrackingCost tracking;
    tracking.setName("tracking");
    auto ref = TRCFileAdapter::read("marker_trajectories.trc");
    TimeSeriesTable refFilt = filterLowpass(ref.flatten(), 6.0, true);
    auto refPacked = refFilt.pack<double>();
    TimeSeriesTableVec3 refToUse(refPacked);

    // Convert from millimeters to meters.
    auto& table = refToUse.updMatrix();
    table = table / 1000.0; 

    // Set marker weights to match IK task weights.
    Set<MarkerWeight> markerWeights;
    markerWeights.cloneAndAppend({ "Top.Head", 3 });
    markerWeights.cloneAndAppend({ "R.ASIS", 3 });
    markerWeights.cloneAndAppend({ "L.ASIS", 3 });
    markerWeights.cloneAndAppend({ "V.Sacral", 3 });
    markerWeights.cloneAndAppend({ "R.Heel", 2 });
    markerWeights.cloneAndAppend({ "R.Toe.Tip", 2 });
    markerWeights.cloneAndAppend({ "L.Heel", 2 });
    markerWeights.cloneAndAppend({ "L.Toe.Tip", 2 });
    MarkersReference markersRef(refToUse, &markerWeights);
    
    tracking.setMarkersReference(markersRef);
    tracking.setAllowUnusedReferences(true);
    mp.addCost(tracking);

    //MucoControlCost effort;
    //effort.setName("effort");
    //mp.addCost(effort);

    // Configure the solver.
    // =====================
    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(100);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_hessian_approximation("exact");

    // Create guess.
    // =============
    MucoIterate guess = ms.createGuess();
    auto model = mp.getPhase().getModel();
    model.initSystem();
    auto statesRef = STOFileAdapter::read("state_reference.mot");
    auto statesRefFilt = filterLowpass(statesRef, 6.0, true);
    model.getSimbodyEngine().convertDegreesToRadians(statesRefFilt);
    STOFileAdapter::write(statesRefFilt, "state_reference_radians.sto");
    guess.setStatesTrajectory(statesRefFilt, true, true);
    ms.setGuess(guess);

    // Solve the problem.
    // ==================
    MucoSolution solution = muco.solve();
    solution.write("sandboxMarkerTrackingWholeBody_marker_solution.sto");

    muco.visualize(solution);

    return solution;
}

/// Solve both problems and compare.
int main() {

    MucoSolution stateTrackingSolution = solveStateTrackingProblem();

    //MucoSolution markerTrackingSolution = solveMarkerTrackingProblem();

    return EXIT_SUCCESS;
}