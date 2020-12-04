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

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Moco/Components/AccelerationMotion.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>

using namespace OpenSim;
using namespace Catch;

template <typename SolverType>
MocoSolution solveDoublePendulumSwingup(const std::string& dynamics_mode) {

    using SimTK::Vec3;

    MocoStudy study;
    study.setName("double_pendulum_swingup_" + dynamics_mode);

    // Define the optimal control problem.
    // ===================================
    MocoProblem& mp = study.updProblem();

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
    mp.setModelAsCopy(model);

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
    // Discourage the model from taking the full time range to complete the 
    // motion.
    auto* ftCost = mp.addGoal<MocoFinalTimeGoal>();
    ftCost->setWeight(0.001);
    // Make sure the end-effector reaches the final target.
    auto* finalCost = mp.addGoal<MocoMarkerFinalGoal>("final");
    finalCost->setWeight(1000.0);
    finalCost->setPointName("/markerset/marker1");
    finalCost->setReferenceLocation(SimTK::Vec3(0, 2, 0));

    // Configure the solver.
    // =====================
    int N = 29; // 29 mesh intervals = 30 mesh points
    auto& solver = study.initSolver<SolverType>();
    solver.set_multibody_dynamics_mode(dynamics_mode);
    solver.set_num_mesh_intervals(N);
    solver.set_transcription_scheme("trapezoidal");
    // solver.set_verbosity(2);

    MocoTrajectory guess = solver.createGuess();
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

    study.print("double_pendulum_swingup_" + dynamics_mode + ".omoco");

    // Solve the problem.
    // ==================
    MocoSolution solution = study.solve();
    // study.visualize(solution);

    return solution;
}

// TODO does not pass consistently on Mac
//TEMPLATE_TEST_CASE("Two consecutive problems produce the same solution", "",
//        MocoCasADiSolver /*, MocoTropterSolver*/) {
//    auto dynamics_mode = GENERATE(as<std::string>{}, "implicit", "explicit");
//    
//    auto solution1 = solveDoublePendulumSwingup<TestType>(dynamics_mode);
//    auto solution2 = solveDoublePendulumSwingup<TestType>(dynamics_mode);
//
//    const double stateError = solution1.compareContinuousVariablesRMS(
//            solution2, {{"states", {}}});
//
//    const double controlError = solution1.compareContinuousVariablesRMS(
//            solution2, {{"controls", {}}});
//
//    CAPTURE(stateError, controlError);
//
//    // Solutions are approximately equal.
//    CHECK(solution1.getFinalTime() ==
//            Approx(solution2.getFinalTime()).margin(1e-2));
//    CHECK(stateError == Approx(0));
//    CHECK(controlError == Approx(0));   
//}

// TODO not passing consistently for MocoTropterSolver on Mac
TEMPLATE_TEST_CASE("Similar solutions between implicit and explicit dynamics",
        "[implicit]", MocoCasADiSolver /*,MocoTropterSolver*/) {
    
    GIVEN("solutions to implicit and explicit problems") {

        auto solutionImplicit =
                solveDoublePendulumSwingup<TestType>("implicit");
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
        MocoTrajectory explicitWithDeriv(solution.getTime(),
                {{"derivatives",
                        {solutionImplicit.getDerivativeNames(), derivTraj}}});
        const double RMS = solutionImplicit.compareContinuousVariablesRMS(
                explicitWithDeriv, {{"derivatives", {}}});
        CAPTURE(RMS);
        CHECK(RMS < 35.0);
    }
}

TEMPLATE_TEST_CASE("Combining implicit dynamics mode with path constraints",
        "[implicit]", MocoCasADiSolver, MocoTropterSolver) {
    class MyPathConstraint : public MocoPathConstraint {
        OpenSim_DECLARE_CONCRETE_OBJECT(MyPathConstraint, MocoPathConstraint);
        void initializeOnModelImpl(
                const Model& model, const MocoProblemInfo&) const override {
            setNumEquations(model.getNumControls());
        }
        void calcPathConstraintErrorsImpl(const SimTK::State& state,
                SimTK::Vector& errors) const override {
            getModel().realizeVelocity(state);
            errors = getModel().getControls(state);
        }
    };
    GIVEN("MocoProblem with path constraints") {
        MocoStudy study;
        auto& prob = study.updProblem();
        auto model = ModelFactory::createPendulum();
        prob.setTimeBounds(0, 1);
        prob.setModelAsCopy(model);
        prob.addGoal<MocoControlGoal>();
        auto* pc = prob.addPathConstraint<MyPathConstraint>();
        MocoConstraintInfo info;
        info.setBounds(std::vector<MocoBounds>(1, {10, 10000}));
        pc->setConstraintInfo(info);
        auto& solver = study.initSolver<TestType>();
        solver.set_multibody_dynamics_mode("implicit");
        const int N = 4;          // mesh intervals
        const int Nc = 2 * N + 1; // collocation points (Hermite-Simpson)
        solver.set_num_mesh_intervals(N);
        MocoSolution solution = study.solve();

        THEN("path constraints are still obeyed") {
            SimTK_TEST_EQ_TOL(solution.getControlsTrajectory(),
                    SimTK::Matrix(Nc, 1, 10.0), 1e-5);
        }
    }
}

TEMPLATE_TEST_CASE("Combining implicit dynamics with kinematic constraints",
        "[implicit][casadi]", /*MocoTropterSolver,*/ MocoCasADiSolver) {
    GIVEN("MocoProblem with a kinematic constraint") {
        MocoStudy study;
        auto& prob = study.updProblem();
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
        prob.setModelAsCopy(model);
        auto& solver = study.initSolver<TestType>();
        solver.set_multibody_dynamics_mode("implicit");
        solver.set_num_mesh_intervals(5);
        solver.set_transcription_scheme("hermite-simpson");
        solver.set_enforce_constraint_derivatives(true);
        MocoSolution solution = study.solve();

        THEN("kinematic constraint is still obeyed") {
            const auto q0value = solution.getStatesTrajectory().col(0);
            const auto q1value = solution.getStatesTrajectory().col(1);
            SimTK_TEST_EQ_TOL(q0value, q1value, 1e-6);
        }
    }
}

SCENARIO("Using MocoTrajectory with the implicit dynamics mode",
        "[implicit][trajectory]") {
    GIVEN("MocoTrajectory with only derivatives") {
        MocoTrajectory trajectory;
        const_cast<SimTK::Matrix*>(&trajectory.getDerivativesTrajectory())
                ->resize(3, 2);
        THEN("it is not empty") { REQUIRE(!trajectory.empty()); }
    }
    GIVEN("MocoTrajectory with only derivative names") {
        MocoTrajectory trajectory;
        const_cast<std::vector<std::string>*>(&trajectory.getDerivativeNames())
                ->resize(3);
        THEN("it is not empty") { REQUIRE(!trajectory.empty()); }
    }
    GIVEN("MocoTrajectory with derivative data") {
        MocoTrajectory iter(createVectorLinspace(6, 0, 1), {}, {}, {},
                {"a", "b"}, {}, {}, {}, {}, {6, 2, 0.5}, {});
        WHEN("calling setNumTimes()") {
            REQUIRE(iter.getDerivativesTrajectory().nrow() != 4);
            iter.setNumTimes(4);
            THEN("setNumTimes() changes number of rows of derivatives") {
                REQUIRE(iter.getDerivativesTrajectory().nrow() == 4);
            }
        }
        WHEN("deserializing") {
            const std::string filename = "testImplicit_MocoTrajectory.sto";
            iter.write(filename);
            THEN("derivatives trajectory is preserved") {
                MocoTrajectory deserialized(filename);
                REQUIRE(iter.getDerivativesTrajectory().nrow() == 6);
                REQUIRE(iter.isNumericallyEqual(deserialized));
            }
        }
    }
    GIVEN("two MocoTrajectorys with different derivative data") {
        const double valueA = 0.5;
        const double valueB = 0.499999;
        MocoTrajectory iterA(createVectorLinspace(6, 0, 1), {}, {}, {},
                {"a", "b"}, {}, {}, {}, {}, {6, 2, valueA}, {});
        MocoTrajectory iterB(createVectorLinspace(6, 0, 1), {}, {}, {},
                {"a", "b"}, {}, {}, {}, {}, {6, 2, valueB}, {});
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

// This class implements a custom component with simple dynamics in implicit
// form.
class MyAuxiliaryImplicitDynamics : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(MyAuxiliaryImplicitDynamics, Component);

public:
    OpenSim_DECLARE_PROPERTY(default_foo, double,
            "Value of the default state returned by initSystem().");
    OpenSim_DECLARE_OUTPUT(implicitresidual_foo, double, getImplicitResidual,
            SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(implicitenabled_foo, bool, getImplicitEnabled,
            SimTK::Stage::Model);
    MyAuxiliaryImplicitDynamics() {
        setName("implicit_auxdyn");
        constructProperty_default_foo(0.5);
    }
    bool getImplicitEnabled(const SimTK::State&) const {
        return true;
    }
    double getImplicitResidual(const SimTK::State& s) const {
        if (!isCacheVariableValid(s, "implicitresidual_foo")) {
            // Get the derivative control value.
            // TODO add method to Component get with index instead.
            const double derivative =
                    getDiscreteVariableValue(s, "implicitderiv_foo");
            // Get the state variable value.
            const double statevar = getStateVariableValue(s, "foo");
            // The dynamics residual: y y' - 1 = 0.
            double residual = derivative * statevar - 1;
            // Update the cache variable with the residual value.
            setCacheVariableValue(s, "implicitresidual_foo", residual);
            markCacheVariableValid(s, "implicitresidual_foo");
        }
        return getCacheVariableValue<double>(s, "implicitresidual_foo");
    }

private:
    void extendInitStateFromProperties(SimTK::State& s) const override {
        Super::extendInitStateFromProperties(s);
        setStateVariableValue(s, "foo", get_default_foo());
    }
    void extendSetPropertiesFromState(const SimTK::State& s) override {
        Super::extendSetPropertiesFromState(s);
        set_default_foo(getStateVariableValue(s, "foo"));
    }
    void computeStateVariableDerivatives(const SimTK::State& s) const override {
        const double derivative =
                getDiscreteVariableValue(s, "implicitderiv_foo");
        setStateVariableDerivativeValue(s, "foo", derivative);
    }
    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        Super::extendAddToSystem(system);
        addStateVariable("foo");
        addDiscreteVariable("implicitderiv_foo", SimTK::Stage::Dynamics);
        addCacheVariable(
                "implicitresidual_foo", double(0), SimTK::Stage::Dynamics);
    }
};

TEST_CASE("Implicit auxiliary dynamics") {
    SECTION("State unit tests") {
        Model model;
        auto* implicit_auxdyn = new MyAuxiliaryImplicitDynamics();
        model.addComponent(implicit_auxdyn);
        auto state = model.initSystem();
        model.finalizeConnections();

        // Create quadratic polynomial function.
        SimTK::Function::Polynomial polyFunc(
                SimTK::Vector(SimTK::Vec3(1, 0, 0)));
        auto time = createVectorLinspace(20, 0, 1);
        for (int i = 0; i < time.size(); ++i) {
            // Set the velocity control discrete variable.
            const double derivativeControl =
                    polyFunc.calcValue(SimTK::Vector(1, time[i]));
            implicit_auxdyn->setDiscreteVariableValue(
                    state, "implicitderiv_foo", derivativeControl);
            model.realizeDynamics(state);

            double statevar =
                    implicit_auxdyn->getStateVariableValue(state, "foo");
            double derivative =
                    implicit_auxdyn->getStateVariableDerivativeValue(
                            state, "foo");

            // y y' = 1
            double residual = statevar * derivative - 1;
            // The check that the residual is being computed correctly.
            CHECK(residual == implicit_auxdyn->getOutputValue<double>(
                                      state, "implicitresidual_foo"));
        }
    }

#ifdef OPENISM_WITH_CASADI
    SECTION("Direct collocation implicit") {
       MocoStudy study;
       auto& problem = study.updProblem();
       auto model = OpenSim::make_unique<Model>();
       model->addComponent(new MyAuxiliaryImplicitDynamics());
       problem.setModel(std::move(model));
       problem.setTimeBounds(0, 1);
       problem.setStateInfo("/implicit_auxdyn/foo", {0, 3}, 1.0);
       auto solution = study.solve();
       const int N = solution.getNumTimes();
       const double final = solution.getStatesTrajectory().getElt(N-1, 0);
       // Correct answer obtained from Matlab with ode45.
       CHECK(final == Approx(1.732).margin(1e-3));
    }
#endif

    SECTION("MocoTropterSolver does not support implicit auxiliary dynamics") {
        MocoStudy study;
        auto& problem = study.updProblem();
        auto model = OpenSim::make_unique<Model>();
        model->addComponent(new MyAuxiliaryImplicitDynamics());
        problem.setModel(std::move(model));
        problem.setTimeBounds(0, 1);
        problem.setStateInfo("/implicit_auxdyn/foo", {0, 3}, 1.0);
        study.initTropterSolver();
        CHECK_THROWS(study.solve(),
                Catch::Contains("MocoTropterSolver does not support problems "
                                "with implicit auxiliary dynamics."));
    }
}
