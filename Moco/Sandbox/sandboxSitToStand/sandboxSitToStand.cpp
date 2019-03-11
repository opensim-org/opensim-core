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
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>

using namespace OpenSim;
using SimTK::Vec3;
using SimTK::Inertia;
using SimTK::Transform;

 /// Convenience function to apply an CoordinateActuator to the model.
void addCoordinateActuator(Model& model, std::string coordName,
    double optimalForce) {

    auto& coordSet = model.updCoordinateSet();

    auto* actu = new CoordinateActuator();
    actu->setName("tau_" + coordName);
    actu->setCoordinate(&coordSet.get(coordName));
    actu->setOptimalForce(1);
    actu->setMinControl(-optimalForce);
    actu->setMaxControl(optimalForce);
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
                "/forceset/" + name).getFiberForceLengthCurve());
        fflc.set_strain_at_one_norm_force(100000);
        fflc.set_stiffness_at_low_force(0.00000001);
        fflc.set_stiffness_at_one_norm_force(0.0001);
        fflc.set_curviness(0);
        model.updComponent<Millard2012EquilibriumMuscle>(
            "/forceset/" + name).setFiberForceLengthCurve(fflc);
    }
}

class UprightCost : public MocoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(UprightCost, MocoCost);
public: 
    UprightCost() {}
protected:
    void calcIntegralCostImpl(const SimTK::State& state,
            SimTK::Real& cost) const override {
        const auto& pelvis = getModel().getBodySet().get("pelvis");
        const auto& R = pelvis.getTransformInGround(state).R();

        const auto& x_dir =
            R.getAxisUnitVec(SimTK::CoordinateAxis::getCoordinateAxis(0));
        const auto& y_dir = 
            R.getAxisUnitVec(SimTK::CoordinateAxis::getCoordinateAxis(1));
        const auto& z_dir =
            R.getAxisUnitVec(SimTK::CoordinateAxis::getCoordinateAxis(2));

        SimTK::UnitVec3 xhat_dir(1, 0, 0);
        SimTK::UnitVec3 yhat_dir(0, 1, 0);
        SimTK::UnitVec3 zhat_dir(0, 0, 1);

        cost = pow(SimTK::dot(x_dir, xhat_dir) - 1, 2) +
               pow(SimTK::dot(y_dir, yhat_dir) - 1, 2) +
               pow(SimTK::dot(z_dir, zhat_dir) - 1, 2);
    }
};

Model createModel(const std::string& actuatorType) {

    Model model("Rajagopal2015_bottom_up_one_leg.osim");
    for (int m = 0; m < model.getMuscles().getSize(); ++m) {
        auto& musc = model.updMuscles().get(m);
        musc.set_ignore_activation_dynamics(true);
        musc.set_ignore_tendon_compliance(true);
        musc.setOptimalForce(10*musc.getOptimalForce());
    }
    minimizePassiveFiberForces(model);
    model.finalizeConnections();
    model.finalizeFromProperties();
    //removeMuscles(model);

    return model;
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
    MocoProblem& mp = moco.updProblem();
    Model model = createModel(opt.actuatorType);
    mp.setModelCopy(model);

    // Set bounds.
    mp.setTimeBounds(0, 1);
    mp.setStateInfo("/jointset/hip_r/hip_flexion_r/value", {-3, 3}, -1, 0);
    mp.setStateInfo("/jointset/hip_r/hip_adduction_r/value", {-1, 1}, -0.5, 0);
    mp.setStateInfo("/jointset/hip_r/hip_rotation_r/value", {-1, 1}, -0.5, 0);
    mp.setStateInfo("/jointset/walker_knee_r/knee_angle_r/value", {-3, 0}, -1, 0);
    mp.setStateInfo("/jointset/ankle_r/ankle_angle_r/value", {-2, 2}, -0.5, 0);

    auto* effort = mp.addCost<MocoControlCost>();
    effort->setName("control_effort");

    mp.addCost<UprightCost>();

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
    ms.set_optim_finite_difference_scheme("forward");

    // Create guess.
    // -------------
    auto guess = ms.createGuess("bounds");
    ms.setGuess(guess);

    MocoSolution solution = moco.solve().unseal();
    moco.visualize(solution);

    return solution;
}

int main() {

    // Set options.
    Options opt;
    opt.num_mesh_points = 10;
    opt.solver = "ipopt";
    opt.constraint_tol = 1e-4;
    opt.convergence_tol = 1e-4;
    opt.dynamics_mode = "implicit";
    opt.max_iterations = 10;

    // Predictive problem.
    MocoSolution torqueSolEffortCasADi = minimizeControlEffort(opt);


    return EXIT_SUCCESS;
}