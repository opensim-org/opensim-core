/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleSlidingMassAdvanced.cpp                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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

/// Translate a point mass in one dimension in minimum time. This example
/// also exhibits more advanced features of Moco, including defining a custom
/// goal (cost term).
///
/// @verbatim
/// minimize   t_f
/// subject to xdot = v
///            vdot = F/m
///            x(0)   = 0
///            x(t_f) = 1
///            v(0)   = 0
///            v(t_f) = 0
/// w.r.t.     x   in [-5, 5]    position of mass
///            v   in [-50, 50]  speed of mass
///            F   in [-50, 50]  force applied to the mass
///            t_f in [0, 5]     final time
/// constants  m       mass
/// @endverbatim

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>

using namespace OpenSim;

std::unique_ptr<Model> createSlidingMassModel() {
    auto model = make_unique<Model>();
    model->setName("sliding_mass");
    model->set_gravity(SimTK::Vec3(0, 0, 0));
    auto* body = new Body("body", 2.0, SimTK::Vec3(0), SimTK::Inertia(0));
    model->addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("slider", model->getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model->addComponent(joint);

    auto* actu = new CoordinateActuator();
    actu->setCoordinate(&coord);
    actu->setName("actuator");
    actu->setOptimalForce(1);
    model->addComponent(actu);

    body->attachGeometry(new Sphere(0.05));

    model->finalizeConnections();

    return model;
}

/// Minimize the sum of squared controls integrated over the phase.
/// (This can be achieved with MocoControlGoal, but we duplicate the cost
/// just to illustrate creating a custom goal).
/// Refer to Doxygen documentation for more information on creating a custom
/// goal.
class MocoCustomEffortGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoCustomEffortGoal, MocoGoal);

public:
    MocoCustomEffortGoal() {}
    MocoCustomEffortGoal(std::string name) : MocoGoal(std::move(name)) {}
    MocoCustomEffortGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {}

protected:
    Mode getDefaultModeImpl() const override { return Mode::Cost; }
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    void initializeOnModelImpl(const Model&) const override {
        setRequirements(1, 1);
    }
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override {
        getModel().realizeVelocity(input.state);
        const auto& controls = getModel().getControls(input.state);
        integrand = controls.normSqr();
    }
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.integral;
    }
};

int main() {

    MocoStudy study;
    study.setName("sliding_mass");

    // Define the optimal control problem.
    // ===================================
    MocoProblem& problem = study.updProblem();

    // Model (dynamics).
    // -----------------
    problem.setModel(createSlidingMassModel());

    // Bounds.
    // -------
    // Initial time must be 0, final time can be within [0, 5].
    problem.setTimeBounds(MocoInitialBounds(0), MocoFinalBounds(0, 5));

    // Initial position must be 0, final position must be 1.
    problem.setStateInfo("/slider/position/value", MocoBounds(-5, 5),
            MocoInitialBounds(0), MocoFinalBounds(1));
    // Initial and final speed must be 0. Use compact syntax.
    problem.setStateInfo("/slider/position/speed", {-50, 50}, 0, 0);

    // Applied force must be between -50 and 50.
    problem.setControlInfo("/actuator", MocoBounds(-50, 50));

    // Cost.
    // -----
    problem.addGoal<MocoFinalTimeGoal>("time");
    problem.addGoal<MocoCustomEffortGoal>("effort");

    // Configure the solver.
    // =====================
    MocoCasADiSolver& solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_ipopt_print_level(4);
    solver.set_optim_max_iterations(50);
    solver.set_optim_finite_difference_scheme("forward");
    solver.set_optim_sparsity_detection("random");
    solver.set_optim_write_sparsity("sliding_mass");
    solver.set_optim_constraint_tolerance(1e-5);
    solver.set_optim_convergence_tolerance(1e-5);

    // Specify an initial guess.
    // -------------------------
    MocoTrajectory guess = solver.createGuess("bounds");
    guess.resampleWithNumTimes(2);
    guess.setTime({0, 0.5});
    guess.setState("/slider/position/value", {0.0, 1.0});
    solver.setGuess(guess);

    // Solve the problem.
    // ==================
    MocoSolution solution = study.solve();
    std::cout << "Solution status: " << solution.getStatus() << std::endl;
    study.visualize(solution);

    return EXIT_SUCCESS;
}
