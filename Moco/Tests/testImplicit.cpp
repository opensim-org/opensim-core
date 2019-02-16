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
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <Moco/osimMoco.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>

// TODO: Test with hermite-simpson.
// TODO: Test with kinematic constraints.

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
    //solver.set_verbosity(2);

    MocoIterate guess = solver.createGuess();
    guess.resampleWithNumTimes(2);
    guess.setTime({0, 1});
    guess.setState("/jointset/j0/q0/value", {0, -SimTK::Pi});
    guess.setState("/jointset/j1/q1/value", {0, 2*SimTK::Pi});
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
        "[implicit]", MocoTropterSolver /*, TODO MocoCasADiSolver*/ ) {
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
                solutionImplicit.compareContinuousVariablesRMS(solution,
                        {}, /* controlNames: */ {"none"}, {"none"},
                        /* derivativeNames: */ {"none"});

        // There is more deviation in the controls.
        const double controlError =
                solutionImplicit.compareContinuousVariablesRMS(solution,
                        {"none"}, /* controlNames: */ {}, {"none"}, {"none"});

        CAPTURE(stateError, controlError);

        // Solutions are approximately equal.
        CHECK(solutionImplicit.getFinalTime() ==
                Approx(solution.getFinalTime()).margin(1e-2));
        CHECK(stateError < 2.0);
        CHECK(controlError < 30.0);

        // Accelerations are correct.
        auto table = solution.exportToStatesTable();
        GCVSplineSet splines(table,
                {"/jointset/j0/q0/speed", "/jointset/j1/q1/speed"});
        OpenSim::Array<double> explicitAccel;
        SimTK::Matrix derivTraj(
                (int)table.getIndependentColumn().size(), 2);
        int i = 0;
        for (const auto& explicitTime : table.getIndependentColumn()) {
            splines.evaluate(explicitAccel, 1, explicitTime);
            derivTraj(i, 0) = explicitAccel[0];
            derivTraj(i, 1) = explicitAccel[1];
            ++i;
        }
        SimTK::Matrix empty;
        MocoIterate explicitWithDeriv(solution.getTime(),
                {}, {}, {}, solutionImplicit.getDerivativeNames(), {},
                empty, empty, empty, derivTraj, SimTK::RowVector());
        const double RMS = solutionImplicit.compareContinuousVariablesRMS(
                explicitWithDeriv, {"none"}, {"none"}, {"none"}, {});
        CAPTURE(RMS);
        CHECK(RMS < 35.0);
    }
}

TEMPLATE_TEST_CASE("Combining implicit dynamics mode with path constraints",
        "[implicit]", MocoTropterSolver /* TODO , MocoCasADiSolver */ ) {
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
        solver.set_num_mesh_points(5);
        MocoSolution solution = moco.solve();

        THEN("path constraints are still obeyed") {
            OpenSim_REQUIRE_MATRIX_TOL(solution.getControlsTrajectory(),
                    SimTK::Matrix(5, 1, 10.0), 1e-5);
        }
    }
}

SCENARIO("Using MocoIterate with the implicit dynamics mode",
        "[implicit][iterate]") {
    GIVEN("MocoIterate with only derivatives") {
        MocoIterate iterate;
        const_cast<SimTK::Matrix*>(&iterate.
                getDerivativesTrajectory())->resize(3, 2);
        THEN("it is not empty") {
            REQUIRE(!iterate.empty());
        }
    }
    GIVEN("MocoIterate with only derivative names") {
        MocoIterate iterate;
        const_cast<std::vector<std::string>*>(&iterate.
                getDerivativeNames())->resize(3);
        THEN("it is not empty") {
            REQUIRE(!iterate.empty());
        }
    }
    GIVEN("MocoIterate with derivative data") {
        MocoIterate iter(createVectorLinspace(6, 0, 1),
                {}, {}, {}, {"a", "b"}, {},
                {}, {}, {}, {6, 2, 0.5}, {});
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
        MocoIterate iterA(createVectorLinspace(6, 0, 1),
                {}, {}, {}, {"a", "b"}, {},
                {}, {}, {}, {6, 2, valueA}, {});
        MocoIterate iterB(createVectorLinspace(6, 0, 1),
                {}, {}, {}, {"a", "b"}, {},
                {}, {}, {}, {6, 2, valueB}, {});
        THEN("not numerically equal") {
            REQUIRE(!iterA.isNumericallyEqual(iterB));
        }
        THEN("RMS error is computed correctly") {
            REQUIRE(iterA.compareContinuousVariablesRMS(iterB) ==
                    Approx(valueA - valueB));
        }
    }
}

TEMPLATE_TEST_CASE("Solving a problem with acceleration-level quantities",
        "[implicit]", MocoTropterSolver /* TODO , MocoCasADiSolver */ ) {
    MocoTool moco;
    moco.updProblem().setModelCopy(ModelFactory::createPendulum());
    auto& solver = moco.initSolver<TestType>();
    solver.set_dynamics_mode("implicit");
    solver.set_num_mesh_points(5);

    class AccelerationIntegralCost : public MocoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(AccelerationIntegralCost, MocoCost);
        void calcIntegralCostImpl(const SimTK::State& state,
                SimTK::Real& cost) const override {
            getModel().realizeAcceleration(state);
            cost = state.getYDot().norm();
        }
    };

    GIVEN("an integral MocoCost that invokes realizeAcceleration") {
        moco.updProblem().addCost<AccelerationIntegralCost>();

        THEN("problem cannot be solved") {
            REQUIRE_THROWS_WITH(moco.solve(),
                    Contains("Cannot realize to Acceleration in implicit "
                             "dynamics mode."));
        }
    }

    class AccelerationEndpointCost : public MocoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(AccelerationEndpointCost, MocoCost);
        void calcEndpointCostImpl(const SimTK::State& state,
                SimTK::Real& cost) const override {
            getModel().realizeAcceleration(state);
            cost = state.getYDot().norm();
        }
    };

    GIVEN("an endpoint MocoCost that invokes realizeAcceleration") {
        moco.updProblem().addCost<AccelerationEndpointCost>();

        THEN("problem cannot be solved") {
            REQUIRE_THROWS_WITH(moco.solve(),
                    Contains("Cannot realize to Acceleration in implicit "
                             "dynamics mode."));
        }
    }

    class AccelerationConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(AccelerationConstraint,
            MocoPathConstraint);
        void initializeOnModelImpl(const Model&) const override {
            setNumEquations(1);
        }
        void calcPathConstraintErrorsImpl(const SimTK::State& state,
                SimTK::Vector& errors) const override {
            getModel().realizeAcceleration(state);
            errors = 0;
        }
    };

    GIVEN("a MocoPathConstraint that invokes realizeAcceleration") {
        moco.updProblem().addPathConstraint<AccelerationConstraint>();

        THEN("problem cannot be solved") {
            REQUIRE_THROWS_WITH(moco.solve(),
                    Contains("Cannot realize to Acceleration in implicit "
                             "dynamics mode."));
        }
    }
}
