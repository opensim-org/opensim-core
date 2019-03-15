/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxSitToStand.cpp                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

#include <Moco/osimMoco.h>

#include <OpenSim/Actuators/osimActuators.h>
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>

using namespace OpenSim;
using SimTK::Inertia;
using SimTK::Transform;
using SimTK::Vec3;

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

/// This essentially removes the effect of passive muscle fiber forces from the
/// model.
void minimizePassiveFiberForces(Model& model) {
    const auto& muscleSet = model.getMuscles();
    Array<std::string> muscNames;
    muscleSet.getNames(muscNames);
    for (int i = 0; i < muscNames.size(); ++i) {
        const auto& name = muscNames.get(i);
        FiberForceLengthCurve fflc(
                model.getComponent<Millard2012EquilibriumMuscle>(
                             "/forceset/" + name)
                        .getFiberForceLengthCurve());
        fflc.set_strain_at_one_norm_force(100000);
        fflc.set_stiffness_at_low_force(0.00000001);
        fflc.set_stiffness_at_one_norm_force(0.0001);
        fflc.set_curviness(0);
        model.updComponent<Millard2012EquilibriumMuscle>("/forceset/" + name)
                .setFiberForceLengthCurve(fflc);
    }
}

Model createModel(const std::string& actuatorType) {

    Model model("sitToStand_3dof9musc.osim");

    if (actuatorType == "torques") {
        removeMuscles(model);
        addCoordinateActuator(model, "hip_flexion_r", 250);
        addCoordinateActuator(model, "knee_angle_r", 500);
        addCoordinateActuator(model, "ankle_angle_r", 250);
    } else if (actuatorType == "Millard2012EquilibriumMuscle") {
        for (int m = 0; m < model.getMuscles().getSize(); ++m) {
            auto& musc = model.updMuscles().get(m);
            musc.set_ignore_activation_dynamics(true);
            musc.set_ignore_tendon_compliance(true);
            // musc.setOptimalForce(10*musc.getOptimalForce());
        }
        ////minimizePassiveFiberForces(model);
        ////model.finalizeConnections();
        ////model.finalizeFromProperties();
    } else if (actuatorType == "DeGrooteFregly2016Muscle") {
        model.finalizeConnections();
        DeGrooteFregly2016Muscle::replaceMuscles(model);
        for (int m = 0; m < model.getMuscles().getSize(); ++m) {
            auto& musc = model.updMuscles().get(m);
            musc.set_ignore_activation_dynamics(true);
            musc.set_ignore_tendon_compliance(true);
            //musc.set_max_isometric_force(10*musc.get_max_isometric_force());
        }
    } else {
        throw std::runtime_error("Unrecognized actuatorType.");
    }

    return model;
}

void setBounds(MocoProblem& mp) {
    mp.setTimeBounds(0, 1);

    mp.setStateInfo("/jointset/hip_r/hip_flexion_r/value", {-2.094, 0.524}, 
        -2, 0);
    mp.setStateInfo("/jointset/knee_r/knee_angle_r/value", {-2.094, 0},
        -2, 0);
    mp.setStateInfo("/jointset/ankle_r/ankle_angle_r/value", {-0.524, 0.698}, 
        -0.5, 0);
    //mp.setStateInfo("/jointset/hip_r/hip_flexion_r/speed", {-50, 50}, {-50, 50}, 0);
    //mp.setStateInfo("/jointset/walker_knee_r/knee_angle_r/speed", {-50, 50}, 
    //    {-50, 50}, 0);
    //mp.setStateInfo("/jointset/ankle_r/ankle_angle_r/speed", {-50, 50}, 
    //    {-50, 50}, 0);
}

struct Options {
    std::string actuatorType = "torques";
    int num_mesh_points = 10;
    double convergence_tol = 1e-2;
    double constraint_tol = 1e-2;
    int max_iterations = 100000;
    std::string hessian_approximation = "limited-memory";
    std::string solver = "ipopt";
    std::string dynamics_mode = "explicit";
    TimeSeriesTable controlsGuess = {};
    MocoIterate previousSolution = {};
};

MocoSolution minimizeControlEffort(const Options& opt) {
    MocoTool moco;
    moco.setName("sandboxSitToStand_minimizeControlEffort");

    MocoProblem& mp = moco.updProblem();
    Model model = createModel(opt.actuatorType);
    mp.setModelCopy(model);

    // Set bounds.
    setBounds(mp);

    auto* effort = mp.addCost<MocoControlCost>();
    effort->setName("control_effort");
    //effort->setWeight("tau_hip_flexion_r", 100);
    //effort->setWeight("tau_knee_angle_r", 100);
    //effort->setWeight("tau_ankle_angle_r", 100);

    // Set solver options.
    // -------------------
    auto& ms = moco.initCasADiSolver();
    ms.set_num_mesh_points(opt.num_mesh_points);
    ms.set_verbosity(2);
    ms.set_dynamics_mode(opt.dynamics_mode);
    ms.set_optim_convergence_tolerance(opt.convergence_tol);
    ms.set_optim_constraint_tolerance(opt.constraint_tol);
    ms.set_optim_solver(opt.solver);
    ms.set_transcription_scheme("hermite-simpson");
    ms.set_optim_max_iterations(opt.max_iterations);
    ms.set_enforce_constraint_derivatives(true);
    ms.set_optim_hessian_approximation(opt.hessian_approximation);
    ms.set_optim_finite_difference_scheme("central");

    // Create guess.
    // -------------
    auto guess = ms.createGuess("bounds");
    ms.setGuess(guess);

    MocoSolution solution = moco.solve().unseal();
    moco.visualize(solution);

    return solution;
}

MocoSolution stateTracking(const Options& opt) {
    MocoTool moco;
    moco.setName("sandboxSitToStand_stateTracking");

    MocoProblem& mp = moco.updProblem();
    Model model = createModel(opt.actuatorType);

    // Get previous solution.
    MocoIterate prevSol = opt.previousSolution;

    // Get states trajectory from previous solution. Need to set the problem
    // model and call initSystem() to create the table internally.
    model.initSystem();
    mp.setModelCopy(model);
    TimeSeriesTable prevStateTraj =
            prevSol.exportToStatesTrajectory(mp).exportToTable(model);
    GCVSplineSet prevStateSpline(prevStateTraj, prevSol.getStateNames());

    setBounds(mp);

    auto* tracking = mp.addCost<MocoStateTrackingCost>();
    tracking->setName("tracking");
    tracking->setReference(prevStateTraj);
    // Don't track coordinates enforced by constraints.
    tracking->setWeight(
            "/jointset/patellofemoral_r/knee_angle_r_beta/value", 0);
    tracking->setWeight(
            "/jointset/patellofemoral_r/knee_angle_r_beta/speed", 0);

    // Need this to recover the correct controls
    //auto* effort = mp.addCost<MocoControlCost>();
    //effort->setName("effort");
    //effort->set_weight(0.01);

    // Set solver options.
    // -------------------
    auto& ms = moco.initCasADiSolver();
    ms.set_num_mesh_points(opt.num_mesh_points);
    ms.set_verbosity(2);
    ms.set_dynamics_mode(opt.dynamics_mode);
    ms.set_optim_convergence_tolerance(opt.convergence_tol);
    ms.set_optim_constraint_tolerance(opt.constraint_tol);
    ms.set_optim_solver(opt.solver);
    ms.set_transcription_scheme("hermite-simpson");
    ms.set_optim_max_iterations(opt.max_iterations);
    ms.set_enforce_constraint_derivatives(true);
    ms.set_optim_hessian_approximation(opt.hessian_approximation);
    ms.set_optim_finite_difference_scheme("central");

    // Create guess.
    // -------------
    ms.setGuess(opt.previousSolution);

    // Solve.
    // ------
    MocoSolution solution = moco.solve().unseal();
    moco.visualize(solution);

    return solution;
}

void compareTrackingToPrediction(const MocoSolution& predictiveSolution,
        const MocoSolution& trackingSolution) {

    std::cout << "Predictive versus tracking comparison" << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "States RMS error: ";
    std::cout << trackingSolution.compareContinuousVariablesRMS(
            predictiveSolution, {{"states", {}}});
    std::cout << std::endl;
    std::cout << "Controls RMS error: ";
    std::cout << trackingSolution.compareContinuousVariablesRMS(
            predictiveSolution, {{"controls", {}}});
    std::cout << std::endl;
    if (trackingSolution.getMultiplierNames().size() != 0) {
        std::cout << "Multipliers RMS error: ";
        std::cout << trackingSolution.compareContinuousVariablesRMS(
                predictiveSolution, {{"multipliers", {}}});
        std::cout << std::endl;
    }
    if (trackingSolution.getDerivativeNames().size() != 0) {
        std::cout << "Derivatives RMS error: ";
        std::cout << trackingSolution.compareContinuousVariablesRMS(
            predictiveSolution, {{"derivatives",{}}});
        std::cout << std::endl;
    }
}

int main() {

    // Set options.
    Options opt;
    // TODO problems with Millard muscles are quite slow
    opt.actuatorType = "torques";
    opt.num_mesh_points = 10;
    opt.constraint_tol = 1e-2;
    opt.convergence_tol = 1e-3;
    opt.solver = "ipopt";
    //opt.max_iterations = 100;
    opt.dynamics_mode = "implicit";

    // Predictive problem.
    MocoSolution torqueSolEffort = minimizeControlEffort(opt);

    opt.previousSolution = torqueSolEffort;
    //MocoSolution torqueSolTrack = stateTracking(opt);

    //compareTrackingToPrediction(torqueSolEffort, torqueSolTrack);

    return EXIT_SUCCESS;
}
