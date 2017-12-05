/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: sandboxMarkerTrackingContactWholeBody.cpp                *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco, Chris Dembia                                   *
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

const bool useActivationCoordinateActuators = false;

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

class /*TODO OSIMMUSCOLLO_API*/AckermannVanDenBogert2010Force : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(AckermannVanDenBogert2010Force, Force);
public:
    OpenSim_DECLARE_PROPERTY(stiffness, double, "TODO N/m^3");
    OpenSim_DECLARE_PROPERTY(dissipation, double, "TODO s/m");
    OpenSim_DECLARE_PROPERTY(friction_coefficient, double, "TODO");
    // TODO rename to transition_velocity
    OpenSim_DECLARE_PROPERTY(tangent_velocity_scaling_factor, double, "TODO");

    OpenSim_DECLARE_OUTPUT(force_on_station, SimTK::Vec3, calcContactForce,
            SimTK::Stage::Velocity);

    OpenSim_DECLARE_SOCKET(station, Station, "TODO");

    AckermannVanDenBogert2010Force() {
        constructProperties();
    }

    /// Compute the force applied to body to which the station is attached, at
    /// the station, expressed in ground.
    SimTK::Vec3 calcContactForce(const SimTK::State& s) const {
        SimTK::Vec3 force(0);
        const auto& pt = getConnectee<Station>("station");
        const auto& pos = pt.getLocationInGround(s);
        const auto& vel = pt.getVelocityInGround(s);
        const SimTK::Real y = pos[1];
        const SimTK::Real velNormal = vel[1];
        // TODO should project vel into ground.
        const SimTK::Real velSliding = vel[0];
        const SimTK::Real depth = 0 - y;
        const SimTK::Real depthRate = 0 - velNormal;
        const SimTK::Real a = get_stiffness();
        const SimTK::Real b = get_dissipation();
        if (depth > 0) {
            force[1] = fmax(0, a * pow(depth, 3) * (1 + b * depthRate));
        }
        const SimTK::Real voidStiffness = 1.0; // N/m
        force[1] += voidStiffness * depth;

        const SimTK::Real velSlidingScaling =
                get_tangent_velocity_scaling_factor();
        const SimTK::Real z0 = exp(-velSliding / velSlidingScaling);
        // TODO decide direction!!!

        const SimTK::Real frictionForce =
                -(1 - z0) / (1 + z0) * get_friction_coefficient() * force[1];

        force[0] = frictionForce;
        return force;
    }
    void computeForce(const SimTK::State& s,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& /*generalizedForces*/) const override {
        const SimTK::Vec3 force = calcContactForce(s);
        const auto& pt = getConnectee<Station>("station");
        const auto& pos = pt.getLocationInGround(s);
        const auto& frame = pt.getParentFrame();
        applyForceToPoint(s, frame, pt.get_location(), force, bodyForces);
        applyForceToPoint(s, getModel().getGround(), pos, -force, bodyForces);
    }

    OpenSim::Array<std::string> getRecordLabels() const override {
        OpenSim::Array<std::string> labels;
        const auto stationName = getConnectee("station").getName();
        labels.append(getName() + "." + stationName + ".force.X");
        labels.append(getName() + "." + stationName + ".force.Y");
        labels.append(getName() + "." + stationName + ".force.Z");
        return labels;
    }
    OpenSim::Array<double> getRecordValues(const SimTK::State& s)
    const override {
        OpenSim::Array<double> values;
        // TODO cache.
        const SimTK::Vec3 force = calcContactForce(s);
        values.append(force[0]);
        values.append(force[1]);
        values.append(force[2]);
        return values;
    }

    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
            const SimTK::State& s,
            SimTK::Array_<SimTK::DecorativeGeometry>& geoms) const override {
        Super::generateDecorations(fixed, hints, s, geoms);
        if (!fixed) {
            getModel().realizeVelocity(s);
            // Normalize contact force vector by body weight so that the line
            // is 1 meter long if the contact force magnitude is equal to
            // body weight.
            const double mg =
                    getModel().getTotalMass(s) * getModel().getGravity().norm();
            // TODO avoid recalculating.
            const auto& pt = getConnectee<Station>("station");
            const auto pt1 = pt.getLocationInGround(s);
            const SimTK::Vec3 force = calcContactForce(s);
            // std::cout << "DEBUGgd force " << force << std::endl;
            const SimTK::Vec3 pt2 = pt1 + force / mg;
            SimTK::DecorativeLine line(pt1, pt2);
            line.setColor(SimTK::Green);
            line.setLineThickness(0.10);
            geoms.push_back(line);

            // TODO move to fixed.
            SimTK::DecorativeSphere sphere;
            sphere.setColor(SimTK::Green);
            sphere.setRadius(0.01);
            sphere.setBodyId(pt.getParentFrame().getMobilizedBodyIndex());
            sphere.setRepresentation(SimTK::DecorativeGeometry::DrawWireframe);
            sphere.setTransform(SimTK::Transform(pt.get_location()));
            geoms.push_back(sphere);
        }
    }

    // TODO potential energy.
private:
    void constructProperties() {
        constructProperty_friction_coefficient(1.0);
        constructProperty_stiffness(5e7);
        constructProperty_dissipation(1.0);
        constructProperty_tangent_velocity_scaling_factor(0.05);
    }
};

// TODO rename ContactForceTracking? ExternalForceTracking?
class MucoForceTrackingCost : public MucoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoForceTrackingCost, MucoCost);
public:
    OpenSim_DECLARE_LIST_PROPERTY(forces, std::string, "TODO");

    MucoForceTrackingCost() {
        constructProperties();
    }
protected:
    void initializeImpl() const override {

    }

    void calcIntegralCostImpl(const SimTK::State& state, double& integrand)
            const override {

    }
private:
    void constructProperties() {
        constructProperty_forces();
    }
};

/// Convenience function to apply an ActivationCoordinateActuator to the model.
void addActivationCoordinateActuator(Model& model, std::string coordName,
    double optimalForce) {

    auto& coordSet = model.updCoordinateSet();

    CoordinateActuator* actu = nullptr;
    if (useActivationCoordinateActuators) {
        auto* actActu = new ActivationCoordinateActuator();
        actActu->set_default_activation(0.1);
        actu = actActu;
    } else {
        actu = new CoordinateActuator();
    }
    actu->setName("tau_" + coordName);
    actu->setCoordinate(&coordSet.get(coordName));
    actu->setOptimalForce(optimalForce);
    model.addComponent(actu);
}

void addContact(Model& model, std::string markerName) {
    const double stiffness = 5e4;
    const double friction_coefficient = 1;
    const double velocity_scaling = 0.05;
    auto* contact = new AckermannVanDenBogert2010Force();
    contact->setName(markerName + "_contact");
    contact->set_stiffness(stiffness);
    contact->set_friction_coefficient(friction_coefficient);
    contact->set_tangent_velocity_scaling_factor(velocity_scaling);
    model.addComponent(contact);
    contact->updSocket("station").setConnecteeName(markerName);
}

/// Set the model and bounds for the specified MucoProblem.
void setModelAndBounds(MucoProblem& mp) {

    Model model("gait1018_subject01_onefoot_v30516.osim");

    //addActivationCoordinateActuator(model, "lumbar_extension", 100);
    //addActivationCoordinateActuator(model, "pelvis_tilt", 100);
    //addActivationCoordinateActuator(model, "pelvis_tx", 1000);
    //addActivationCoordinateActuator(model, "pelvis_ty", 1000);
    //addActivationCoordinateActuator(model, "hip_flexion_r", 100);
    //addActivationCoordinateActuator(model, "knee_angle_r", 100);
    //addActivationCoordinateActuator(model, "ankle_angle_r", 100);
    //addActivationCoordinateActuator(model, "hip_flexion_l", 100);
    //addActivationCoordinateActuator(model, "knee_angle_l", 100);
    //addActivationCoordinateActuator(model, "ankle_angle_l", 100);
    addActivationCoordinateActuator(model, "rz", 40); // TODO 100);
    addActivationCoordinateActuator(model, "tx", 100); // TODO 1000);
    addActivationCoordinateActuator(model, "ty", 100); // TODO 1000);

    const auto& calcn = dynamic_cast<Body&>(model.updComponent("calcn_r"));
    model.addMarker(new Marker("R.Heel.Distal", calcn,
            SimTK::Vec3(0.01548, -0.0272884, -0.00503735)));
    model.addMarker(new Marker("R.Ball.Lat", calcn,
            SimTK::Vec3(0.16769, -0.0272884, 0.066)));
    model.addMarker(new Marker("R.Ball.Med", calcn,
            SimTK::Vec3(0.1898, -0.0272884, -0.03237)));
    addContact(model, "R.Heel.Distal");
    addContact(model, "R.Ball.Lat");
    addContact(model, "R.Ball.Med");
    //addContact(model, "R.Heel");
    //addContact(model, "R.Toe.Lat");
    //addContact(model, "R.Toe.Med");
    //addContact(model, "R.Midfoot.Lat");
    //addContact(model, "R.Midfoot.Sup");
    //addContact(model, "L.Heel");
    //addContact(model, "L.Toe.Lat");
    //addContact(model, "L.Toe.Med");
    //addContact(model, "L.Midfoot.Lat");
    //addContact(model, "L.Midfoot.Sup");

    /*
    // TODO
    auto state = model.initSystem();
    model.updCoordinateSet().get("ty").setValue(state, 0.05);
    SimTK::RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    // Without the next line: Simbody takes steps that are too large and NaNs
    // are generated.
    integrator.setMaximumStepSize(0.05);
    Manager manager(model, state, integrator);
    state = manager.integrate(1.6);
    const auto& statesTimeStepping = manager.getStateStorage();
    visualize(model, statesTimeStepping);

    std::exit(-1); // TODO
    */

    // Model(dynamics).
    // -----------------
    mp.setModel(model);

    // Bounds.
    // -------
    MucoBounds defaultSpeedBounds(-7, 7);
    mp.setTimeBounds(0.48, 1.8); // TODO [.58, 1.8] for gait cycle of right leg.
    mp.setStateInfo("ground_toes/rz/value", { -10, 10 });
    mp.setStateInfo("ground_toes/rz/speed", defaultSpeedBounds);
    mp.setStateInfo("ground_toes/tx/value", { -10, 10 });
    mp.setStateInfo("ground_toes/tx/speed", defaultSpeedBounds);
    mp.setStateInfo("ground_toes/ty/value", { -10, 10 });
    mp.setStateInfo("ground_toes/ty/speed", defaultSpeedBounds);
    //mp.setStateInfo("ground_pelvis/pelvis_tilt/value", { -10, 10 });
    //mp.setStateInfo("ground_pelvis/pelvis_tilt/speed", defaultSpeedBounds);
    //mp.setStateInfo("ground_pelvis/pelvis_tx/value", { -10, 10 });
    //mp.setStateInfo("ground_pelvis/pelvis_tx/speed", defaultSpeedBounds);
    //mp.setStateInfo("ground_pelvis/pelvis_ty/value", { -10, 10 });
    //mp.setStateInfo("ground_pelvis/pelvis_ty/speed", defaultSpeedBounds);
    //mp.setStateInfo("hip_r/hip_flexion_r/value", { -10, 10 });
    //mp.setStateInfo("hip_r/hip_flexion_r/speed", defaultSpeedBounds);
    //mp.setStateInfo("knee_r/knee_angle_r/value", { -10, 10 });
    //mp.setStateInfo("knee_r/knee_angle_r/speed", defaultSpeedBounds);
    //mp.setStateInfo("ankle_r/ankle_angle_r/value", { -10, 10 });
    //mp.setStateInfo("ankle_r/ankle_angle_r/speed", defaultSpeedBounds);
    //mp.setStateInfo("hip_l/hip_flexion_l/value", { -10, 10 });
    //mp.setStateInfo("hip_l/hip_flexion_l/speed", defaultSpeedBounds);
    //mp.setStateInfo("knee_l/knee_angle_l/value", { -10, 10 });
    //mp.setStateInfo("knee_l/knee_angle_l/speed", { -10, 10 });
    //mp.setStateInfo("ankle_l/ankle_angle_l/value", { -10, 10 });
    //mp.setStateInfo("ankle_l/ankle_angle_l/speed", defaultSpeedBounds);
    //mp.setStateInfo("back/lumbar_extension/value", { -10, 10 });
    //mp.setStateInfo("back/lumbar_extension/speed", defaultSpeedBounds);
    if (useActivationCoordinateActuators) {
        mp.setStateInfo("tau_rz/activation", { -1, 1 });
        mp.setStateInfo("tau_tx/activation", { -1, 1 });
        mp.setStateInfo("tau_ty/activation", { -1, 1 });
        //mp.setStateInfo("tau_lumbar_extension/activation", { -1, 1 });
        //mp.setStateInfo("tau_pelvis_tilt/activation", {-1, 1});
        //mp.setStateInfo("tau_pelvis_tx/activation", { -1, 1 });
        //mp.setStateInfo("tau_pelvis_ty/activation", { -1, 1 });
        //mp.setStateInfo("tau_hip_flexion_r/activation", { -1, 1 });
        //mp.setStateInfo("tau_knee_angle_r/activation", { -1, 1 });
        //mp.setStateInfo("tau_ankle_angle_r/activation", { -1, 1 });
        //mp.setStateInfo("tau_hip_flexion_l/activation", { -1, 1 });
        //mp.setStateInfo("tau_knee_angle_l/activation", { -1, 1 });
        //mp.setStateInfo("tau_ankle_angle_l/activation", { -1, 1 });
    }
    mp.setControlInfo("tau_rz", { -1, 1 });
    mp.setControlInfo("tau_tx", { -1, 1 });
    mp.setControlInfo("tau_ty", { -1, 1 });
    //mp.setControlInfo("tau_lumbar_extension", { -1, 1 });
    //mp.setControlInfo("tau_pelvis_tilt", { -1, 1 });
    //mp.setControlInfo("tau_pelvis_tx", { -1, 1 });
    //mp.setControlInfo("tau_pelvis_ty", { -1, 1 });
    //mp.setControlInfo("tau_hip_flexion_r", { -1, 1 });
    //mp.setControlInfo("tau_knee_angle_r", { -1, 1 });
    //mp.setControlInfo("tau_ankle_angle_r", { -1, 1 });
    //mp.setControlInfo("tau_hip_flexion_l", { -1, 1 });
    //mp.setControlInfo("tau_knee_angle_l", { -1, 1 });
    //mp.setControlInfo("tau_ankle_angle_l", { -1, 1 });
}

/// Solve a full-body (10 DOF) tracking problem by having each model
/// generalized coordinate track the coordinate value obtained from
/// inverse kinematics.
///
/// Estimated time to solve: ~35 minutes.
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
    auto ref = STOFileAdapter::read("walk_gait1018_state_reference.mot");
    // Convert treadmill data to overground data.
    const auto& reftime = ref.getIndependentColumn();
    ref.updDependentColumn("ground_pelvis/pelvis_ty/value") -= 0.05;
    auto reftx = ref.updDependentColumn("ground_pelvis/pelvis_tx/value");
    // I got this speed with "guess and check" by observing, in
    // visualization, if the foot stayed in the same spot during stance.
    const double walkingSpeed = 1.10; // m/s
    for (int i = 0; i < reftx.nrow(); ++i) {
        reftx[i] += walkingSpeed * reftime[i];
    }
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
    ms.set_dynamics_mode("implicit");

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

    // muco.visualize(guess);

    // Solve the problem.
    // ==================
    MucoSolution solution = muco.solve();
    solution.write("sandboxMarkerTrackingContactWholeBody_states_solution.sto");

    muco.visualize(solution);

    // TODO with barely any contact: 26 minutes to solve (instead of 6)
    // TODO fix contact locations.
    // TODO play with finite difference perturbations.

    return solution;
}

/// Solve a full-body (10 DOF) tracking problem by having the model markers
/// track the marker trajectories directly.
///
/// Estimated time to solve: ~95 minutes.
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
    auto ref = TRCFileAdapter::read("walk_marker_trajectories.trc");
    // Convert from millimeters to meters.
    ref.updMatrix() /= 1000;
    // TODO shift x and y positions to create "overground" trial.
    const auto& reftime = ref.getIndependentColumn();
    const double walkingSpeed = 1.15; // m/s
    for (int i = 0; i < (int)ref.getNumColumns(); ++i) {
        SimTK::VectorView_<SimTK::Vec3> col = ref.updDependentColumnAtIndex(i);
        for (int j = 0; j < col.size(); ++j) {
            col[j][0] += walkingSpeed * reftime[j]; // x
            col[j][1] -= 0.02; // y TODO
        }
    }
    TimeSeriesTable refFilt = filterLowpass(ref.flatten(), 6.0, true);
    auto refPacked = refFilt.pack<double>();
    TimeSeriesTableVec3 refToUse(refPacked);


    // Set marker weights to match IK task weights.
    Set<MarkerWeight> markerWeights;
    //markerWeights.cloneAndAppend({ "Top.Head", 3 });
    //markerWeights.cloneAndAppend({ "R.ASIS", 3 });
    //markerWeights.cloneAndAppend({ "L.ASIS", 3 });
    //markerWeights.cloneAndAppend({ "V.Sacral", 3 });
    markerWeights.cloneAndAppend({ "R.Heel", 2 });
    markerWeights.cloneAndAppend({ "R.Toe.Tip", 2 });
    //markerWeights.cloneAndAppend({ "L.Heel", 2 });
    //markerWeights.cloneAndAppend({ "L.Toe.Tip", 2 });
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
    ms.set_num_mesh_points(50);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_hessian_approximation("exact");
    ms.set_dynamics_mode("implicit");

    // Create guess.
    // =============
    //MucoIterate guess = ms.createGuess();
    //auto model = mp.getPhase().getModel();
    //model.initSystem();
    //auto statesRef = STOFileAdapter::read("walk_gait1018_state_reference.mot");
    //auto statesRefFilt = filterLowpass(statesRef, 6.0, true);
    //model.getSimbodyEngine().convertDegreesToRadians(statesRefFilt);
    //STOFileAdapter::write(statesRefFilt, "state_reference_radians.sto");
    //guess.setStatesTrajectory(statesRefFilt, true, true);
    //ms.setGuess(guess);

    // Solve the problem.
    // ==================
    MucoSolution solution = muco.solve();
    solution.write("sandboxMarkerTrackingContactWholeBody_marker_solution.sto");

    muco.visualize(solution);

    return solution;
}

/// Solve both problems and compare.
int main() {

    // TODO won't work with the time range we're using for the right leg:
    // MucoSolution stateTrackingSolution = solveStateTrackingProblem();

    MucoSolution markerTrackingSolution = solveMarkerTrackingProblem();

    return EXIT_SUCCESS;
}