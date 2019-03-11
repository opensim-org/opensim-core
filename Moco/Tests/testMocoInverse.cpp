/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testMocoInverse.cpp                                          *
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

/// In this file, we attempt to solve
/// This file contains a problem we hope to solve robustly in the future.
/// It serves as a goal.
#include <Moco/osimMoco.h>

#include <OpenSim/Common/LogManager.h>

#define CATCH_CONFIG_MAIN
#include "Testing.h"

using namespace OpenSim;

/// This test case helps debug when it's necessary to calll prescribe() versus
/// realize().
TEST_CASE("PrescribedKinematics prescribe() and realize()") {
    Model model = ModelFactory::createPendulum();
    auto* motion = new PositionMotion();
    const double c2 = 1.3;
    const double c1 = 0.17;
    const double c0 = 0.81;
    motion->setPositionForCoordinate(model.getCoordinateSet().get(0),
            PolynomialFunction(createVector({c2, c1, c0})));
    model.addModelComponent(motion);
    auto state = model.initSystem();

    const auto& system = model.getSystem();
    const auto& y = state.getY();
    const auto& ydot = state.getYDot();
    CHECK(y[0] == 0);
    CHECK(y[1] == 0);
    system.prescribe(state);
    CHECK(y[0] == Approx(c0));
    CHECK(y[1] == Approx(c1));
    model.realizeAcceleration(state);
    CHECK(y[0] == Approx(c0));
    CHECK(y[1] == Approx(c1));
    CHECK(ydot[0] == Approx(c1));
    CHECK(ydot[1] == Approx(2 * c2));

    const double t = 0.3;
    state.setTime(t);
    system.prescribe(state);
    CHECK(y[0] == Approx(c0 + c1 * t + c2 * pow(t, 2)));
    CHECK(y[1] == Approx(c1 + 2 * c2 * t));
    model.realizeAcceleration(state);
    CHECK(y[0] == Approx(c0 + c1 * t + c2 * pow(t, 2)));
    CHECK(y[1] == Approx(c1 + 2 * c2 * t));
    CHECK(ydot[0] == Approx(c1 + 2 * c2 * t));
    CHECK(ydot[1] == Approx(2 * c2));
}

TEST_CASE("PrescribedKinematics direct collocation") {

    // Make sure that custom dynamics are still handled properly even when
    // we are skipping over the kinematic/multibody states. That is, this test
    // ensures we handle indices properly.
    class CustomDynamics : public Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(CustomDynamics, Component);

    public:
        OpenSim_DECLARE_PROPERTY(
                default_s, double, "Default state variable value.");
        CustomDynamics() { constructProperty_default_s(0.4361); }
        void extendAddToSystem(SimTK::MultibodySystem& system) const override {
            Super::extendAddToSystem(system);
            addStateVariable("s");
        }
        void extendInitStateFromProperties(SimTK::State& s) const override {
            Super::extendInitStateFromProperties(s);
            setStateVariableValue(s, "s", get_default_s());
        }
        void extendSetPropertiesFromState(const SimTK::State& s) override {
            Super::extendSetPropertiesFromState(s);
            set_default_s(getStateVariableValue(s, "s"));
        }
        void computeStateVariableDerivatives(
                const SimTK::State& s) const override {
            setStateVariableDerivativeValue(
                    s, "s", getStateVariableValue(s, "s"));
        }
    };
    Model model = ModelFactory::createPendulum();
    model.initSystem();
    auto* motion = new PositionMotion();
    motion->setPositionForCoordinate(
            model.getCoordinateSet().get(0), LinearFunction(1.3, 0.17));
    model.addModelComponent(motion);
    model.addComponent(new CustomDynamics());

    MocoTool moco;
    auto& problem = moco.updProblem();
    problem.setModelCopy(model);
    problem.setTimeBounds(0, 1);
    const double init_s = 0.2;
    problem.setStateInfo("/customdynamics/s", {0, 100}, init_s);
    auto& solver = moco.initCasADiSolver();
    solver.set_dynamics_mode("implicit");

    MocoSolution solution = moco.solve();
    OpenSim_CHECK_MATRIX_TOL(solution.getState("/customdynamics/s"),
            0.2 * SimTK::exp(solution.getTime()), 1e-4);
}

TEST_CASE("MocoInverse gait10dof18musc") {
    std::cout.rdbuf(LogManager::cout.rdbuf());
    std::cerr.rdbuf(LogManager::cerr.rdbuf());

    Model model("testGait10dof18musc_subject01.osim");
    model.finalizeConnections();
    DeGrooteFregly2016Muscle::replaceMuscles(model);

    MocoInverse inverse;
    inverse.setModel(model);
    inverse.set_ignore_activation_dynamics(true);
    inverse.set_ignore_tendon_compliance(true);
    inverse.setKinematicsFile("walk_gait1018_state_reference.mot");
    inverse.set_create_reserve_actuators(2.0);

    inverse.setExternalLoadsFile("walk_gait1018_subject01_grf.xml");

    inverse.solve();

    // TODO: Implement cost minimization directly in CasADi.
    //      -> evaluating the integral cost only takes up like 5% of the
    //         computational time; the only benefit would be accurate
    //         derivatives.
    // TODO: Activation dynamics.: can solve but takes way longer.
    //      Without, solves in 4 seconds.
    // TODO parallelization is changing the number of iterations for a
    // solution.
    // TODO: are results reproducible?? stochasticity in Millard model?
    // TODO: problem scaling.
    // TODO: solve with toy problem (single muscle).
}
