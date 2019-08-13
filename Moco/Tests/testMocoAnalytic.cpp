/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testMocoAnalytic.cpp                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

#define CATCH_CONFIG_MAIN
#include "Testing.h"
#include <Moco/osimMoco.h>

#include <OpenSim/Actuators/SpringGeneralizedForce.h>

using namespace OpenSim;

// class CustomDynamics : public ScalarActuator {
//     OpenSim_DECLARE_CONCRETE_OBJECT(CustomDynamics, ScalarActuator);
//
// public:
//     CustomDynamics() = default;
//     void extendAddToSystem(SimTK::MultibodySystem& system) const override {
//         Super::extendAddToSystem(system);
//         addStateVariable("x0");
//         addStateVariable("x1");
//     }
//     void extendInitStateFromProperties(SimTK::State& s) const override {
//         Super::extendInitStateFromProperties(s);
//         setStateVariableValue(s, "x0", 0);
//         setStateVariableValue(s, "x1", 0);
//     }
//     void computeStateVariableDerivatives(const SimTK::State& s) const
//     override {
//         const auto x1 = getStateVariableValue(s, "x1");
//         setStateVariableDerivativeValue(s, "x0", x1);
//         setStateVariableDerivativeValue(s, "x1", x1 + getControl(s));
//     }
//     double computeActuation(const SimTK::State&) const override { return 0; }
// };

/// Taken from Kirk 1998 equations 5.1-69 and 5.1-70, page 199.
SimTK::Matrix expectedSolution(const SimTK::Vector& time) {
    using std::exp;
    SimTK::Mat22 A(-2 - 0.5 * exp(-2) + 0.5 * exp(2),
            1 - 0.5 * exp(-2) - 0.5 * exp(2), -1 + 0.5 * exp(-2) + 0.5 * exp(2),
            0.5 * exp(-2) - 0.5 * exp(2));
    SimTK::Vec2 b(5, 2);
    SimTK::Vec2 c = A.invert() * b;
    const double c2 = c[0];
    const double c3 = c[1];

    auto x0_func = [&c2, &c3](const double& t) -> double {
        return c2 * (-t - 0.5 * exp(-t) + 0.5 * exp(t)) +
               c3 * (1 - 0.5 * exp(-t) - 0.5 * exp(t));
    };

    auto x1_func = [&c2, &c3](const double& t) -> double {
        return c2 * (-1 + 0.5 * exp(-t) + 0.5 * exp(t)) +
               c3 * (0.5 * exp(-t) - 0.5 * exp(t));
    };

    SimTK::Matrix expectedStatesTrajectory(time.size(), 2);
    for (int itime = 0; itime < time.size(); ++itime) {
        expectedStatesTrajectory(itime, 0) = x0_func(time[itime]);
        expectedStatesTrajectory(itime, 1) = x1_func(time[itime]);
    }
    return expectedStatesTrajectory;
}

TEMPLATE_TEST_CASE("Second order linear min effort",
        ""
        // , MocoTropterSolver Too slow.
        ,
        MocoCasADiSolver) {
    // Kirk 1998, Example 5.1-1, page 198.

    Model model;
    // auto* force = new CustomDynamics();
    // force->setName("force");
    // model.addForce(force);
    auto* body = new Body("b", 1, SimTK::Vec3(0), SimTK::Inertia(0));
    model.addBody(body);

    auto* joint = new SliderJoint("j", model.getGround(), *body);
    joint->updCoordinate().setName("coord");
    model.addJoint(joint);

    auto* damper = new SpringGeneralizedForce("coord");
    damper->setViscosity(-1.0);
    model.addForce(damper);

    auto* actu = new CoordinateActuator("coord");
    model.addForce(actu);
    model.finalizeConnections();

    MocoStudy moco;
    auto& problem = moco.updProblem();

    problem.setModelCopy(model);
    problem.setTimeBounds(0, 2);
    problem.setStateInfo("/jointset/j/coord/value", {-10, 10}, 0, 5);
    problem.setStateInfo("/jointset/j/coord/speed", {-10, 10}, 0, 2);
    problem.setControlInfo("/forceset/coordinateactuator", {-50, 50});

    problem.addGoal<MocoControlGoal>("effort", 0.5);

    auto& solver = moco.initSolver<TestType>();
    // solver.set_num_mesh_points(1000);
    // solver.set_transcription_scheme("hermite-simpson");
    solver.set_optim_hessian_approximation("limited-memory");
    // MocoSolution solution = moco.solve();

    // SimTK::Matrix error = solution.getStatesTrajectory() -
    //         expectedStatesTrajectory;
    // std::cout << "DEBUG " << error.norm() << std::endl;

    // OpenSim_CHECK_MATRIX_ABSTOL(solution.getStatesTrajectory(),
    // expectedStatesTrajectory, 1e-3);

    solver.set_verbosity(0);
    for (int N = 10; N < 10000; N *= 2) {
        solver.set_num_mesh_points(N);
        const auto solution = moco.solve();
        const auto expectedStatesTrajectory =
                expectedSolution(solution.getTime());
        SimTK::Matrix error =
                solution.getStatesTrajectory() - expectedStatesTrajectory;
        std::cout << "DEBUG " << N << " " << error.normRMS() << std::endl;
    }
    // TODO plot solution match and runtime.
}

// class ForceDoesNoWork : public MocoPathConstraint {
//     OpenSim_DECLARE_CONCRETE_OBJECT(ForceDoesNoWork, MocoPathConstraint);
// public:
//     void initializeOnModelImpl(const Model&) const override {
//         setNumEquations(1);
//     }
//     void calcPathConstraintErrorsImpl(const SimTK::State& state,
//             SimTK::Vector& errors) const override {
//         const auto& controls = getModel().getControls(state);
//         const SimTK::Vec2 force(controls[0], controls[1]);
//         const SimTK::Vec2 velocity(state.getU()[0], state.getU()[1]);
//         errors[0] = SimTK::dot(force, velocity);
//     }
//
// };

TEMPLATE_TEST_CASE("Brachistochrone", ""
        // , MocoTropterSolver Too slow.
        ,
        MocoCasADiSolver) {

    Model model = ModelFactory::createBrachistochrone();

    MocoStudy moco;
    auto& problem = moco.updProblem();

    problem.setModelCopy(model);
    problem.setTimeBounds(0, {0, 10.0});
    problem.setStateInfo("/brachistochrone/x", {-10, 10}, 0, 1);
    problem.setStateInfo("/brachistochrone/y", {-10, 10}, 0);
    problem.setStateInfo("/brachistochrone/v", {-10, 10}, 0);
    problem.setControlInfo("/brachistochrone", {-10, 10});

    problem.addGoal<MocoFinalTimeGoal>();

    auto& solver = moco.initSolver<TestType>();
    // solver.set_num_mesh_points(1000);
    // solver.set_transcription_scheme("hermite-simpson");
    solver.set_optim_hessian_approximation("limited-memory");
    solver.set_parallel(0); // TODO: causes controls to be bogus, and takes
    // a lot longer to solve, if parallelization is disabled.
    MocoSolution solution = moco.solve();

    std::cout << solution.getStatesTrajectory() << std::endl;
    std::cout << solution.getControlsTrajectory() << std::endl;

    // SimTK::Matrix error = solution.getStatesTrajectory() -
    //         expectedStatesTrajectory;
    // std::cout << "DEBUG " << error.norm() << std::endl;

    // OpenSim_CHECK_MATRIX_ABSTOL(solution.getStatesTrajectory(),
    // expectedStatesTrajectory, 1e-3);

    // solver.set_verbosity(0);
    // for (int N = 10; N < 10000; N *= 2) {
    //     solver.set_num_mesh_points(N);
    //     const auto solution = moco.solve();
    //     // const auto expectedStatesTrajectory =
    //     //         expectedSolution(solution.getTime());
    //     SimTK::Matrix error =
    //             solution.getStatesTrajectory() - expectedStatesTrajectory;
    //     std::cout << "DEBUG " << N << " " << error.normRMS() << std::endl;
    // }
    // TODO plot solution match and runtime.
}
