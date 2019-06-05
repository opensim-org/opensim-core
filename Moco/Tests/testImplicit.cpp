/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testImplicit.cpp                                             *
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
#define CATCH_CONFIG_MAIN
#include "Testing.h"
#include <Moco/Components/AccelerationMotion.h>
#include <Moco/osimMoco.h>

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Common/LogManager.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>

using namespace OpenSim;
using namespace Catch;

template <typename SolverType>
MocoSolution solveDoublePendulumSwingup(const std::string& dynamics_mode) {

    using SimTK::Vec3;

    MocoTool moco;
    moco.setName("double_pendulum_swingup_" + dynamics_mode);

    // Define the optimal control problem.
    // ===================================
    MocoProblem& mp = moco.updProblem();

    // Model (dynamics).
    // -----------------
    auto model = ModelFactory::createDoublePendulum();
    Sphere target(0.1);
    target.setColor(SimTK::Red);
    PhysicalOffsetFrame* targetframe = new PhysicalOffsetFrame(
            "targetframe", model.getGround(), SimTK::Transform(Vec3(0, 2, 0)));
    model.updGround().addComponent(targetframe);
    targetframe->attachGeometry(target.clone());

    Sphere start(target);
    PhysicalOffsetFrame* startframe = new PhysicalOffsetFrame(
            "startframe", model.getGround(), SimTK::Transform(Vec3(2, 0, 0)));
    model.updGround().addComponent(startframe);
    start.setColor(SimTK::Green);
    startframe->attachGeometry(start.clone());
    model.finalizeConnections();
    mp.setModelCopy(model);

    // Bounds.
    // -------
    mp.setTimeBounds(0, {0, 5});
    mp.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0);
    mp.setStateInfo("/jointset/j0/q0/speed", {-50, 50}, 0, 0);
    mp.setStateInfo("/jointset/j1/q1/value", {-10, 10}, 0);
    mp.setStateInfo("/jointset/j1/q1/speed", {-50, 50}, 0, 0);
    mp.setControlInfo("/tau0", {-100, 100});
    mp.setControlInfo("/tau1", {-100, 100});

    // Cost.
    // -----
    auto* ftCost = mp.addCost<MocoFinalTimeCost>();
    ftCost->set_weight(0.001);

    auto* endpointCost = mp.addCost<MocoMarkerEndpointCost>("endpoint");
    endpointCost->set_weight(1000.0);
    endpointCost->setPointName("/markerset/marker1");
    endpointCost->setReferenceLocation(SimTK::Vec3(0, 2, 0));

    // Configure the solver.
    // =====================
    int N = 30;
    auto& solver = moco.initSolver<SolverType>();
    solver.set_dynamics_mode(dynamics_mode);
    solver.set_num_mesh_points(N);
    solver.set_transcription_scheme("trapezoidal");
    // solver.set_verbosity(2);

    MocoIterate guess = solver.createGuess();
    guess.resampleWithNumTimes(2);
    guess.setTime({0, 1});
    guess.setState("/jointset/j0/q0/value", {0, -SimTK::Pi});
    guess.setState("/jointset/j1/q1/value", {0, 2 * SimTK::Pi});
    guess.setState("/jointset/j0/q0/speed", {0, 0});
    guess.setState("/jointset/j1/q1/speed", {0, 0});
    guess.setControl("/tau0", {0, 0});
    guess.setControl("/tau1", {0, 0});
    guess.resampleWithNumTimes(10);
    solver.setGuess(guess);

    // moco.visualize(guess);

    moco.print("double_pendulum_swingup_" + dynamics_mode + ".omoco");

    // Solve the problem.
    // ==================
    MocoSolution solution = moco.solve();
    // moco.visualize(solution);

    return solution;
}

TEMPLATE_TEST_CASE("Similar solutions between implicit and explicit dynamics",
        "[implicit]", MocoTropterSolver, MocoCasADiSolver) {
    std::cout.rdbuf(LogManager::cout.rdbuf());
    std::cerr.rdbuf(LogManager::cerr.rdbuf());
    GIVEN("solutions to implicit and explicit problems") {

        auto solutionImplicit =
                solveDoublePendulumSwingup<TestType>("implicit");
        // TODO: Solving the implicit problem multiple times gives different
        // results; https://github.com/stanfordnmbl/moco/issues/172.
        // auto solutionImplicit2 = solveDoublePendulumSwingup("implicit");
        // TODO: The solution to this explicit problem changes every time.
        auto solution = solveDoublePendulumSwingup<TestType>("explicit");

        CAPTURE(solutionImplicit.getFinalTime(), solution.getFinalTime());

        const double stateError =
                solutionImplicit.compareContinuousVariablesRMS(
                        solution, {{"states", {}}});

        // There is more deviation in the controls.
        const double controlError =
                solutionImplicit.compareContinuousVariablesRMS(
                        solution, {{"controls", {}}});

        CAPTURE(stateError, controlError);

        // Solutions are approximately equal.
        CHECK(solutionImplicit.getFinalTime() ==
                Approx(solution.getFinalTime()).margin(1e-2));
        CHECK(stateError < 2.0);
        CHECK(controlError < 30.0);

        // Accelerations are correct.
        auto table = solution.exportToStatesTable();
        GCVSplineSet splines(
                table, {"/jointset/j0/q0/speed", "/jointset/j1/q1/speed"});
        OpenSim::Array<double> explicitAccel;
        SimTK::Matrix derivTraj((int)table.getIndependentColumn().size(), 2);
        int i = 0;
        for (const auto& explicitTime : table.getIndependentColumn()) {
            splines.evaluate(explicitAccel, 1, explicitTime);
            derivTraj(i, 0) = explicitAccel[0];
            derivTraj(i, 1) = explicitAccel[1];
            ++i;
        }
        MocoIterate explicitWithDeriv(solution.getTime(),
                {{"derivatives",
                        {solutionImplicit.getDerivativeNames(), derivTraj}}});
        const double RMS = solutionImplicit.compareContinuousVariablesRMS(
                explicitWithDeriv, {{"derivatives", {}}});
        CAPTURE(RMS);
        CHECK(RMS < 35.0);
    }
}

TEMPLATE_TEST_CASE("Combining implicit dynamics mode with path constraints",
        "[implicit]", MocoTropterSolver, MocoCasADiSolver) {
    std::cout.rdbuf(LogManager::cout.rdbuf());
    std::cerr.rdbuf(LogManager::cerr.rdbuf());
    class MyPathConstraint : public MocoPathConstraint {
        OpenSim_DECLARE_CONCRETE_OBJECT(MyPathConstraint, MocoPathConstraint);
        void initializeOnModelImpl(const Model& model) const override {
            setNumEquations(model.getNumControls());
        }
        void calcPathConstraintErrorsImpl(const SimTK::State& state,
                SimTK::Vector& errors) const override {
            errors = getModel().getControls(state);
        }
    };
    GIVEN("MocoProblem with path constraints") {
        MocoTool moco;
        auto& prob = moco.updProblem();
        auto model = ModelFactory::createPendulum();
        prob.setTimeBounds(0, 1);
        prob.setModelCopy(model);
        prob.addCost<MocoControlCost>();
        auto* pc = prob.addPathConstraint<MyPathConstraint>();
        MocoConstraintInfo info;
        info.setBounds(std::vector<MocoBounds>(1, {10, 10000}));
        pc->setConstraintInfo(info);
        auto& solver = moco.initSolver<TestType>();
        solver.set_dynamics_mode("implicit");
        const int N = 5; // mesh points
        const int Nc = 2*N - 1; // collocation points (Hermite-Simpson)
        solver.set_num_mesh_points(N);
        MocoSolution solution = moco.solve();

        THEN("path constraints are still obeyed") {
            SimTK_TEST_EQ_TOL(solution.getControlsTrajectory(), 
                SimTK::Matrix(Nc, 1, 10.0), 1e-5);
        }
    }
}

TEMPLATE_TEST_CASE("Combining implicit dynamics with kinematic constraints",
        "[implicit]", /*MocoTropterSolver,*/ MocoCasADiSolver) {
    std::cout.rdbuf(LogManager::cout.rdbuf());
    std::cerr.rdbuf(LogManager::cerr.rdbuf());
    GIVEN("MocoProblem with a kinematic constraint") {
        MocoTool moco;
        auto& prob = moco.updProblem();
        auto model = ModelFactory::createDoublePendulum();
        prob.setTimeBounds(0, 1);
        auto* constraint = new CoordinateCouplerConstraint();
        Array<std::string> names;
        names.append("q0");
        constraint->setIndependentCoordinateNames(names);
        constraint->setDependentCoordinateName("q1");
        LinearFunction func(1.0, 0.0);
        constraint->setFunction(func);
        model.addConstraint(constraint);
        prob.setModelCopy(model);
        auto& solver = moco.initSolver<TestType>();
        solver.set_dynamics_mode("implicit");
        solver.set_num_mesh_points(5);
        solver.set_transcription_scheme("hermite-simpson");
        solver.set_enforce_constraint_derivatives(true);
        MocoSolution solution = moco.solve();

        THEN("kinematic constraint is still obeyed") {
            const auto q0value = solution.getStatesTrajectory().col(0);
            const auto q1value = solution.getStatesTrajectory().col(1);
            SimTK_TEST_EQ_TOL(q0value, q1value, 1e-6);
        }
    }
}

SCENARIO("Using MocoIterate with the implicit dynamics mode",
        "[implicit][iterate]") {
    GIVEN("MocoIterate with only derivatives") {
        MocoIterate iterate;
        const_cast<SimTK::Matrix*>(&iterate.getDerivativesTrajectory())
                ->resize(3, 2);
        THEN("it is not empty") { REQUIRE(!iterate.empty()); }
    }
    GIVEN("MocoIterate with only derivative names") {
        MocoIterate iterate;
        const_cast<std::vector<std::string>*>(&iterate.getDerivativeNames())
                ->resize(3);
        THEN("it is not empty") { REQUIRE(!iterate.empty()); }
    }
    GIVEN("MocoIterate with derivative data") {
        MocoIterate iter(createVectorLinspace(6, 0, 1), {}, {}, {}, {"a", "b"},
                {}, {}, {}, {}, {6, 2, 0.5}, {});
        WHEN("calling setNumTimes()") {
            REQUIRE(iter.getDerivativesTrajectory().nrow() != 4);
            iter.setNumTimes(4);
            THEN("setNumTimes() changes number of rows of derivatives") {
                REQUIRE(iter.getDerivativesTrajectory().nrow() == 4);
            }
        }
        WHEN("deserializing") {
            const std::string filename = "testImplicit_MocoIterate.sto";
            iter.write(filename);
            THEN("derivatives trajectory is preserved") {
                MocoIterate deserialized(filename);
                REQUIRE(iter.getDerivativesTrajectory().nrow() == 6);
                REQUIRE(iter.isNumericallyEqual(deserialized));
            }
        }
    }
    GIVEN("two MocoIterates with different derivative data") {
        const double valueA = 0.5;
        const double valueB = 0.499999;
        MocoIterate iterA(createVectorLinspace(6, 0, 1), {}, {}, {}, {"a", "b"},
                {}, {}, {}, {}, {6, 2, valueA}, {});
        MocoIterate iterB(createVectorLinspace(6, 0, 1), {}, {}, {}, {"a", "b"},
                {}, {}, {}, {}, {6, 2, valueB}, {});
        THEN("not numerically equal") {
            REQUIRE(!iterA.isNumericallyEqual(iterB));
        }
        THEN("RMS error is computed correctly") {
            REQUIRE(iterA.compareContinuousVariablesRMS(iterB) ==
                    Approx(valueA - valueB));
        }
    }
}

TEST_CASE("AccelerationMotion") {
    Model model = OpenSim::ModelFactory::createNLinkPendulum(1);
    AccelerationMotion* accel = new AccelerationMotion("motion");
    model.addModelComponent(accel);
    auto state = model.initSystem();
    state.updQ()[0] = -SimTK::Pi / 2;
    model.realizeAcceleration(state);
    // Default.
    CHECK(state.getUDot()[0] == Approx(0).margin(1e-10));

    // Enable.
    accel->setEnabled(state, true);
    SimTK::Vector udot(1);
    udot[0] = SimTK::Random::Uniform(-1, 1).getValue();
    accel->setUDot(state, udot);
    model.realizeAcceleration(state);
    CHECK(state.getUDot()[0] == Approx(udot[0]).margin(1e-10));

    // Disable.
    accel->setEnabled(state, false);
    model.realizeAcceleration(state);
    CHECK(state.getUDot()[0] == Approx(0).margin(1e-10));
}
