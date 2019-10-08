/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxMarkerTrackingContactWholeBody.cpp                    *
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
#include <Moco/osimMoco.h>

#include <MocoSandboxShared.h>

const bool useActivationCoordinateActuators = false;

using namespace OpenSim;

/// Convenience function to apply an ActivationCoordinateActuator to the model.
void addCoordinateActuator(Model& model, std::string coordName,
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

void addContact(Model& model, std::string markerName, const double stiffness = 5e7) {
    //const double stiffness = 5e7;
    const double friction_coefficient = 0.95;
    const double velocity_scaling = 0.3;
    auto* contact = new AckermannVanDenBogert2010Force();
    contact->setName(markerName + "_contact");
    contact->set_stiffness(stiffness);
    contact->set_friction_coefficient(friction_coefficient);
    contact->set_tangent_velocity_scaling_factor(velocity_scaling);
    model.addComponent(contact);
    contact->updSocket("station").setConnecteeName(markerName);
}

/// Set the model and bounds for the specified MocoProblem.
void setModelAndBounds(MocoProblem& mp, bool useOptimizedModel = false) {

    std::string modelFile;
    if (useOptimizedModel) {
        modelFile = "gait1018_subject01_onefoot_v30516_opt.osim";
    } else {
        modelFile = "gait1018_subject01_onefoot_v30516.osim";
    }
    Model model(modelFile);

    //addCoordinateActuator(model, "lumbar_extension", 100);
    //addCoordinateActuator(model, "pelvis_tilt", 100);
    //addCoordinateActuator(model, "pelvis_tx", 1000);
    //addCoordinateActuator(model, "pelvis_ty", 1000);
    //addCoordinateActuator(model, "hip_flexion_r", 100);
    //addCoordinateActuator(model, "knee_angle_r", 100);
    //addCoordinateActuator(model, "ankle_angle_r", 100);
    //addCoordinateActuator(model, "hip_flexion_l", 100);
    //addCoordinateActuator(model, "knee_angle_l", 100);
    //addCoordinateActuator(model, "ankle_angle_l", 100);
    addCoordinateActuator(model, "rz", 250); // TODO 100);
    addCoordinateActuator(model, "tx", 5000); // TODO 1000);
    addCoordinateActuator(model, "ty", 5000); // TODO 1000);

    if (!useOptimizedModel) {
        const auto& calcn = dynamic_cast<Body&>(model.updComponent("calcn_r"));
        model.addMarker(new Marker("R.Heel.Distal", calcn,
                SimTK::Vec3(0.01548, -0.0272884, -0.00503735)));
        model.addMarker(new Marker("R.Ball.Lat", calcn,
                SimTK::Vec3(0.16769, -0.0272884, 0.066)));
        model.addMarker(new Marker("R.Ball.Med", calcn,
                SimTK::Vec3(0.1898, -0.0272884, -0.03237)));
        addContact(model, "R.Heel.Distal", 5e7);
        addContact(model, "R.Ball.Lat", 7.5e7);
        addContact(model, "R.Ball.Med", 7.5e7);
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
    }

    // TODO
    /*
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
    MocoBounds defaultSpeedBounds(-25, 25);
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
MocoSolution solveStateTrackingProblem() {

    MocoStudy study;
    study.setName("whole_body_state_tracking");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& mp = study.updProblem();

    // Bounds and model.
    setModelAndBounds(mp);

    // Cost.
    // -----
    MocoStateTrackingCost tracking;
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
    mp.addGoal(tracking);

    // Configure the solver.
    // =====================
    MocoTropterSolver& ms = study.initTropterSolver();
    ms.set_num_mesh_intervals(50);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_multibody_dynamics_mode("implicit");

    // Create guess.
    // =============
    MocoTrajectory guess = ms.createGuess();
    auto model = mp.getPhase().getModel();
    model.initSystem();
    auto refFilt2 = refFilt;
    model.getSimbodyEngine().convertDegreesToRadians(refFilt2);
    STOFileAdapter::write(refFilt2, "state_reference_radians.sto");
    guess.setStatesTrajectory(refFilt2, true, true);
    ms.setGuess(guess);

    // moco.visualize(guess);

    // Solve the problem.
    // ==================
    MocoSolution solution = study.solve();
    solution.write("sandboxMarkerTrackingContactWholeBody_states_solution.sto");

    study.visualize(solution);

    // TODO with barely any contact: 26 minutes to solve (instead of 6)
    // TODO fix contact locations.
    // TODO play with finite difference perturbations.

    return solution;
}

/// Solve a full-body (10 DOF) tracking problem by having the model markers
/// track the marker trajectories directly.
///
/// Estimated time to solve: ~95 minutes.
MocoSolution solveMarkerTrackingProblem(bool createGuess,
        bool useOptimizedModel) {

    MocoStudy study;
    study.setName("whole_body_marker_tracking");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& mp = study.updProblem();


    // Bounds and model.
    setModelAndBounds(mp, useOptimizedModel);

    auto modelCopy = mp.getPhase().getModel();
    modelCopy.initSystem();

    // Cost.
    // -----
    MocoMarkerTrackingCompCost tracking;
    tracking.setName("tracking");
    auto ref = TRCFileAdapter::read("marker_trajectories.trc");
    // Convert from millimeters to meters.
    ref.updMatrix() /= 1000;
    // TODO shift x and y positions to create "overground" trial.
    const auto& reftime = ref.getIndependentColumn();
    const double walkingSpeed = 1.15; // m/s
    for (int i = 0; i < (int)ref.getNumColumns(); ++i) {
        SimTK::VectorView_<SimTK::Vec3> col = ref.updDependentColumnAtIndex(i);
        for (int j = 0; j < col.size(); ++j) {
            col[j][0] += walkingSpeed * reftime[j]; // x
            col[j][1] -= 0.03; // y TODO
        }
    }
    TimeSeriesTable refFilt = filterLowpass(ref.flatten({ "_x", "_y", "_z" }), 
        6.0, true);
    STOFileAdapter::write(refFilt,
        "sandboxMarkerTrackingContactWholeBody_marker_trajectories.sto");
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
    tracking.set_weight(1);
    // TODO tracking.set_weight(0.000001);
    tracking.setFreeRadius(0.01);
    tracking.setTrackedMarkerComponents("xy");
    // TODO mp.addGoal(tracking);

    auto data = STOFileAdapter::read("walk_gait1018_subject01_grf.mot");
    auto time = data.getIndependentColumn();
    SimTK::Vector Fx = data.getDependentColumn("ground_force_vx");
    SimTK::Vector Fy = data.getDependentColumn("ground_force_vy");
    GCVSpline splineFx(5, (int) time.size(), time.data(), &Fx[0]);
    GCVSpline splineFy(5, (int) time.size(), time.data(), &Fy[0]);
    if (!createGuess) {
        MocoForceTrackingCost grfTracking;
        // TODO this is a complete hack!
        grfTracking.setName("grf");
        grfTracking.m_refspline_x = splineFx;
        grfTracking.m_refspline_y = splineFy;
        double normGRFs = 0.001;
        double weight = 0.1;
        // TODO double weight = 0.000001;
        grfTracking.set_weight(normGRFs * weight);

        for (const auto& force :
                modelCopy.getComponentList<AckermannVanDenBogert2010Force>()) {
            std::cout << "Contact Force: " << force.getName() << std::endl;
            grfTracking.append_forces(force.getName());
        }
        // TODO horizontal component
        // grfTracking.set_tracked_grf_components("vertical");
        grfTracking.set_tracked_grf_components("horizontal");
        // grfTracking.set_tracked_grf_components("all");
        grfTracking.set_free_force_window(25.0);
        mp.addGoal(grfTracking);
    }

    //MocoControlGoal effort;
    //effort.setName("effort");
    //mp.addGoal(effort);

    // Configure the solver.
    // =====================
    MocoTropterSolver& ms = study.initTropterSolver();
    ms.set_num_mesh_intervals(50);
    ms.set_verbosity(2);
    ms.set_optim_solver("ipopt");
    ms.set_optim_hessian_approximation("limited-memory"); // TODO "exact");
    ms.set_multibody_dynamics_mode("implicit");
    ms.set_optim_max_iterations(5000);
    ms.set_optim_convergence_tolerance(1e-4);
    ms.set_optim_constraint_tolerance(1e-2);

    // Create guess.
    // =============
    if (!createGuess) {
        MocoTrajectory guess("sandboxMarkerTrackingContactWholeBody_guess.sto");
        ms.setGuess(guess);
        // moco.visualize(guess);
    }
    //MocoTrajectory guess = ms.createGuess();
    //auto model = mp.getPhase().getModel();
    //model.initSystem();
    //auto statesRef = STOFileAdapter::read("walk_gait1018_state_reference.mot");
    //auto statesRefFilt = filterLowpass(statesRef, 6.0, true);
    //model.getSimbodyEngine().convertDegreesToRadians(statesRefFilt);
    //STOFileAdapter::write(statesRefFilt, "state_reference_radians.sto");
    //guess.setStatesTrajectory(statesRefFilt, true, true);
    //ms.setGuess(guess);
    // TODO describe how this guess is generated (without contact).

    // Solve the problem.
    // ==================
    MocoSolution solution = study.solve().unseal();
    solution.write("sandboxMarkerTrackingContactWholeBody_marker_solution.sto");
    if (createGuess) {
        solution.write("sandboxMarkerTrackingContactWholeBody_guess.sto");
    }

    // Compute the contact force for the direct collocation solution.
    const auto statesTraj = solution.exportToStatesTrajectory(mp);
    Model model = mp.getPhase().getModel();
    model.initSystem();
    TimeSeriesTableVec3 contactForceHistory;
    contactForceHistory.setColumnLabels({"ground_force"});
    //{"ground_force_sim", "ground_force_exp"});

    const auto contactForces =
            model.getComponentList<AckermannVanDenBogert2010Force>();
    for (const auto& s : statesTraj) {
        model.realizeVelocity(s);
        SimTK::Vec3 sim(0);
        for (const auto& force : contactForces) {
            sim += force.calcContactForceOnStation(s);
        }
        SimTK::Vector time(1, s.getTime());
        SimTK::Vec3 exp(splineFx.calcValue(time), splineFy.calcValue(time), 0);
        contactForceHistory.appendRow(s.getTime(), {sim/*TODO , exp*/});
    }

    STOFileAdapter::write(contactForceHistory.flatten({ "_vx", "_vy", "_vz" }),
        "sandboxMarkerTrackingContactWholeBody_dircol_force.sto");

    // Compute the model marker trajectories for the direct collocation 
    // solution.
    const auto& RHeel = model.getComponent<Marker>("R.Heel");
    const auto& RMidfootSup = model.getComponent<Marker>("R.Midfoot.Sup");
    const auto& RMidfootLat = model.getComponent<Marker>("R.Midfoot.Lat");
    const auto& RToeLat = model.getComponent<Marker>("R.Toe.Lat");
    const auto& RToeMed = model.getComponent<Marker>("R.Toe.Med");
    const auto& RToeTip = model.getComponent<Marker>("R.Toe.Tip");
    TimeSeriesTableVec3 modelMarkerHistory;
    modelMarkerHistory.setColumnLabels({"R.Heel", "R.Midfoot.Sup", 
        "R.Midfoot.Lat", "R.Toe.Lat", "R.Toe.Med", "R.Toe.Tip"});

    for (const auto& s : statesTraj) {
        model.realizePosition(s);
        modelMarkerHistory.appendRow(s.getTime(),
            {RHeel.getLocationInGround(s), 
             RMidfootSup.getLocationInGround(s),
             RMidfootLat.getLocationInGround(s),
             RToeLat.getLocationInGround(s),
             RToeMed.getLocationInGround(s),
             RToeTip.getLocationInGround(s) });
    }

    STOFileAdapter::write(modelMarkerHistory.flatten({ "_x", "_y", "_z" }),
        "sandboxMarkerTrackingContactWholeBody_dircol_markers.sto");

    study.visualize(solution);

    return solution;
}

/// Solve both problems and compare.
int main() {

    Object::registerType(AckermannVanDenBogert2010Force());

    bool createGuess = false;
    bool useOptimizedModel = true;

    /*
    Model model("gait1018_subject01_onefoot_v30516_opt.osim");
    model.setUseVisualizer(true);
    SimTK::State s = model.initSystem();
    model.updCoordinateSet().get("ty").setValue(s, 1.0);
    Manager manager(model, s);
    manager.integrate(10.0);
    std::cin.get();
    std::exit(-1);
    */

    // TODO won't work with the time range we're using for the right leg:
    // MocoSolution stateTrackingSolution = solveStateTrackingProblem();

    MocoSolution markerTrackingSolution =
            solveMarkerTrackingProblem(createGuess, useOptimizedModel);

    return EXIT_SUCCESS;
}
