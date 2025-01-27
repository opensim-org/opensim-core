/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testMocoControllers.cpp                                      *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2024 Stanford University and the Authors                     *
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

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/Control/SynergyController.h>

#include <catch2/catch_all.hpp>
#include "Testing.h"

using namespace OpenSim;

using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

TEMPLATE_TEST_CASE("Sliding mass with PrescribedController", "",
        MocoCasADiSolver) {

    // Solve a sliding mass problem and store the results.
    TimeSeriesTable controlsTable;
    SimTK::Matrix statesTrajectory;
    {
        MocoStudy study;
        auto& problem = study.updProblem();
        Model model = ModelFactory::createSlidingPointMass();
        problem.setModelAsCopy(model);
        problem.setTimeBounds(0, 2);
        problem.setStateInfo("/slider/position/value", {0, 1}, 0, 1);
        problem.setStateInfo("/slider/position/speed", {-100, 100}, 0, 0);
        problem.setControlInfo("/forceset/actuator", {-50, 50});
        problem.addGoal<MocoControlGoal>();
        auto& solver = study.initSolver<TestType>();
        solver.set_num_mesh_intervals(50);
        MocoSolution solution = study.solve();
        solution.write("testMocoControllers_testSlidingMass_solution.sto");
        statesTrajectory = solution.getStatesTrajectory();
        controlsTable = solution.exportToControlsTable();
    }

    // Apply the control from the previous problem to a new problem with a
    // PrescribedController and check that we get the same states trajectory
    // back.
    {
        const auto& time = controlsTable.getIndependentColumn();
        const auto& control =
                controlsTable.getDependentColumn("/forceset/actuator");
        Model model = ModelFactory::createSlidingPointMass();
        auto* controller = new PrescribedController();
        controller->addActuator(
                model.getComponent<Actuator>("/forceset/actuator"));
        controller->prescribeControlForActuator("/forceset/actuator",
                GCVSpline(5, control.size(), time.data(), &control[0],
                        "/forceset/actuator", 0.0));
        model.addController(controller);
        model.finalizeConnections();

        MocoStudy study;
        auto& problem = study.updProblem();
        problem.setModelAsCopy(model);
        problem.setTimeBounds(0, 2);
        problem.setStateInfo("/slider/position/value", {0, 1}, 0);
        problem.setStateInfo("/slider/position/speed", {-100, 100}, 0);
        auto& solver = study.initSolver<TestType>();
        solver.set_num_mesh_intervals(50);
        MocoSolution solution = study.solve();
        // Add the PrescribedController controls to the solution.
        solution.generateControlsFromModelControllers(model);
        solution.write(
                "testMocoControllers_testSlidingMass_prescribed_solution.sto");

        OpenSim_REQUIRE_MATRIX_ABSTOL(solution.getStatesTrajectory(),
            statesTrajectory, 1e-9);

        // We should get back exactly the same controls trajectory that we
        // provided via the PrescribedController.
        OpenSim_REQUIRE_MATRIX_ABSTOL(solution.getControlsTrajectory(),
                controlsTable.getMatrix(), 1e-12);
        REQUIRE(solution.getControlNames() == controlsTable.getColumnLabels());
    }
}

TEST_CASE("MocoControlGoal: ignoring controlled actuators") {

    Model model = ModelFactory::createSlidingPointMass();
    auto* controller = new PrescribedController();
    controller->addActuator(
            model.getComponent<Actuator>("/forceset/actuator"));
    controller->prescribeControlForActuator("/forceset/actuator",
            Constant(1.0));
    model.addController(controller);
    model.finalizeConnections();

    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelAsCopy(model);
    problem.setTimeBounds(0, 1);
    problem.setStateInfo("/slider/position/value", {0, 1}, 0);
    problem.setStateInfo("/slider/position/speed", {-100, 100}, 0);
    auto& solver = study.initCasADiSolver();
    solver.set_optim_max_iterations(1);
    auto* controlGoal = problem.addGoal<MocoControlGoal>();

    SECTION("Default behavior (do not ignore)") {
        MocoSolution solution = study.solve().unseal();
        CHECK(solution.getObjective() == Approx(1.0));
    }

    SECTION("Ignore controlled actuators") {
        controlGoal->setIgnoreControlledActuators(true);
        solver.resetProblem(problem);
        MocoSolution solution = study.solve().unseal();
        CHECK(solution.getObjective() == 0);
    }
}

namespace {

    // Based on ModelFactory::createNLinkPendulum(), but allows us to add the 
    // CoordinateActuators to the ForceSet, which is necessary for some of the 
    // tests below.
    Model createTriplePendulum() {
        Model model; 
        model.setName("triple_pendulum");
        const auto& ground = model.getGround();

        using SimTK::Inertia;
        using SimTK::Vec3;

        Ellipsoid bodyGeometry(0.5, 0.1, 0.1);
        bodyGeometry.setColor(SimTK::Gray);

        const PhysicalFrame* prevBody = &ground;
        for (int i = 0; i < 3; ++i) {
            const std::string istr = std::to_string(i);
            auto* bi = new OpenSim::Body("b" + istr, 1, Vec3(0), Inertia(1));
            model.addBody(bi);

            // Assume each body is 1 m long.
            auto* ji = new PinJoint("j" + istr, *prevBody, Vec3(0), Vec3(0), 
                    *bi, Vec3(-1, 0, 0), Vec3(0));
            auto& qi = ji->updCoordinate();
            qi.setName("q" + istr);
            model.addJoint(ji);

            auto* taui = new CoordinateActuator();
            taui->setCoordinate(&ji->updCoordinate());
            taui->setName("tau" + istr);
            taui->setOptimalForce(1);
            model.addForce(taui);

            auto* marker = new Marker("marker" + istr, *bi, Vec3(0));
            model.addMarker(marker);

            // Attach an ellipsoid to a frame located at the center of each body.
            PhysicalOffsetFrame* bicenter = new PhysicalOffsetFrame(
                    "b" + istr + "center", *bi, 
                    SimTK::Transform(Vec3(-0.5, 0, 0)));
            bi->addComponent(bicenter);
            bicenter->attachGeometry(bodyGeometry.clone());

            prevBody = bi;
        }

        model.finalizeConnections();
        return model;
    }

    Model createControlledTriplePendulumModel(bool controllerEnabled = true) {
        Model model = createTriplePendulum();
        auto* controller = new SynergyController();
        controller->setName("synergy_controller");
        controller->addActuator(
                model.getComponent<CoordinateActuator>("/forceset/tau0"));
        controller->addActuator(
                model.getComponent<CoordinateActuator>("/forceset/tau1"));
        controller->addActuator(
                model.getComponent<CoordinateActuator>("/forceset/tau2"));
        controller->addSynergyVector(createVector({0.25, 0.75, 0.5}));
        controller->addSynergyVector(createVector({0.5, 0.5, 0.25}));
        controller->setEnabled(controllerEnabled);
        model.addController(controller);
        model.finalizeConnections();
        return model;
    }

    template<typename SolverType = MocoCasADiSolver>
    MocoStudy createTriplePendulumMocoStudy(const Model& model,
            bool ignoreControlledActuators = false,
            bool ignoreInputControls = false) {
        MocoStudy study;
        auto& problem = study.updProblem(); 
        problem.setModelAsCopy(model);
        problem.setTimeBounds(0, 0.5);
        problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0, -0.5);
        problem.setStateInfo("/jointset/j0/q0/speed", {-50, 50}, 0);
        problem.setStateInfo("/jointset/j1/q1/value", {-10, 10}, 0);
        problem.setStateInfo("/jointset/j1/q1/speed", {-50, 50}, 0);
        problem.setStateInfo("/jointset/j2/q2/value", {-10, 10}, 0);
        problem.setStateInfo("/jointset/j2/q2/speed", {-50, 50}, 0);
        MocoBounds bounds(-100, 100);
        const auto& controller = model.getComponent<SynergyController>(
                "/controllerset/synergy_controller");
        if (controller.isEnabled()) {
            problem.setInputControlInfo(
                "/controllerset/synergy_controller/synergy_excitation_0", 
                bounds);
            problem.setInputControlInfo(
                "/controllerset/synergy_controller/synergy_excitation_1", 
                bounds);
        } else {
            problem.setControlInfo("/forceset/tau0", bounds);
            problem.setControlInfo("/forceset/tau1", bounds);
            problem.setControlInfo("/forceset/tau2", bounds);
        }

        auto* effort = problem.addGoal<MocoControlGoal>();
        effort->setName("effort");
        effort->setIgnoreControlledActuators(ignoreControlledActuators);
        effort->setIgnoreInputControls(ignoreInputControls);
        auto& solver = study.initSolver<SolverType>();
        solver.set_num_mesh_intervals(50);
        return study;
    }
}

TEST_CASE("InputController behavior") {
    auto model = createControlledTriplePendulumModel(); 

    SECTION("No Inputs connected (controls are default)") {
        SimTK::State state = model.initSystem();
        model.realizeVelocity(state);
        SimTK::Vector defaults = SimTK::Test::randVector(3);
        model.updDefaultControls() = defaults;

        SimTK::Vector controls = model.getControls(state);
        SimTK_TEST_EQ(defaults, controls);
    }

    SECTION("Incorrect number of Inputs connected") {
        SignalGenerator* constant = new SignalGenerator();
        constant->set_function(Constant(1.0));
        model.addComponent(constant);

        auto& controller = model.updComponent<SynergyController>(
                "/controllerset/synergy_controller");
        controller.connectInput_controls(constant->getOutput("signal"));

        CHECK_THROWS_WITH(model.finalizeConnections(),
                ContainsSubstring("Expected 2 Input connectee(s)"));
    }

    SECTION("Correct number of Inputs connected") {
        SimTK::Vector constants = SimTK::Test::randVector(2);
        SignalGenerator* constant0 = new SignalGenerator();
        constant0->set_function(Constant(constants[0]));
        model.addComponent(constant0);
        SignalGenerator* constant1 = new SignalGenerator();
        constant1->set_function(Constant(constants[1]));
        model.addComponent(constant1);

        auto& controller = model.updComponent<SynergyController>(
                "/controllerset/synergy_controller");
        controller.connectInput_controls(constant0->getOutput("signal"));
        controller.connectInput_controls(constant1->getOutput("signal"));
        model.finalizeConnections();
        SimTK::State state = model.initSystem();
        model.realizeVelocity(state);


        SimTK::Matrix synergyVectors = controller.getSynergyVectorsAsMatrix();
        SimTK::Vector expected = synergyVectors * constants;
        SimTK::Vector controls = model.getControls(state);
        SimTK_TEST_EQ(expected, controls);
    }
}

TEST_CASE("MocoTrajectory::generateControlsFromModelControllers()") {
    Model model = createTriplePendulum();
    model.initSystem();
    auto* controller = new PrescribedController();
    controller->setName("prescribed_controller");
    controller->addActuator(
            model.getComponent<CoordinateActuator>("/forceset/tau0"));
    controller->addActuator(
            model.getComponent<CoordinateActuator>("/forceset/tau1"));
    controller->addActuator(
            model.getComponent<CoordinateActuator>("/forceset/tau2"));
    SimTK::Real tau0 = SimTK::Test::randReal();
    SimTK::Real tau1 = SimTK::Test::randReal();
    SimTK::Real tau2 = SimTK::Test::randReal();
    controller->prescribeControlForActuator("/forceset/tau0", Constant(tau0));
    controller->prescribeControlForActuator("/forceset/tau1", Constant(tau1));
    controller->prescribeControlForActuator("/forceset/tau2", Constant(tau2));
    model.addController(controller);
    model.finalizeConnections();

    std::vector<std::string> stateNames = {"/jointset/j0/q0/value",
                                           "/jointset/j1/q1/value",
                                           "/jointset/j2/q2/value", 
                                           "/jointset/j0/q0/speed",
                                           "/jointset/j1/q1/speed",
                                           "/jointset/j2/q2/speed"};
    SimTK::Matrix states = SimTK::Test::randMatrix(1, 6);
    MocoTrajectory trajectory(SimTK::Vector(1, 0.0), stateNames, {}, {}, {}, 
            states, SimTK::Matrix(), SimTK::Matrix(), SimTK::RowVector());

    trajectory.generateControlsFromModelControllers(model);
    const auto& controls = trajectory.getControlsTrajectory();
    CHECK(controls.nrow() == 1);
    CHECK(controls.ncol() == 3);

    // Approx() is necessary because the controls are generated from the 
    // controllers and then inserted into the MocoTrajectory
    CHECK(controls(0, 0) == tau0);
    CHECK(controls(0, 1) == tau1);
    CHECK(controls(0, 2) == tau2);
}

TEMPLATE_TEST_CASE("Triple pendulum with synergy-like InputController", "",
        MocoCasADiSolver) {
    Model model = createControlledTriplePendulumModel();
    model.initSystem();

    MocoStudy study = createTriplePendulumMocoStudy<TestType>(model);
    MocoSolution solution = study.solve();
    solution.write("testMocoControllers_testTriplePendulum_solution.sto");
    //study.visualize(solution);

    SECTION("Solution size is correct") {
        CHECK(solution.getNumInputControls() == 2);
        CHECK(solution.getNumControls() == 0);
    }

    SECTION("Timestepping produces the same states trajectory") {
        MocoTrajectory traj = 
                simulateTrajectoryWithTimeStepping(solution, model);
        const double error = solution.compareContinuousVariablesRMS(
                traj, {{"states", {}}});
        CHECK(error < 1e-3);
    }

    SECTION("Solution is resuable as guess") {
        auto& solver = study.updSolver<TestType>();
        solver.setGuess(solution);
    }

    SECTION("Calling analyzeMocoTrajectory() is invalid") {
        std::string expected = "MocoUtilities::analyzeMocoTrajectory(): The "
                "number of controls in the MocoTrajectory does not match";
        CHECK_THROWS_WITH(
            analyzeMocoTrajectory<double>(model, solution, {"output"}), 
            ContainsSubstring(expected));
    }

    solution.generateControlsFromModelControllers(model);
    SECTION("Model controllers produce correct controls") {
        CHECK(solution.getNumControls() == 3);
        TimeSeriesTable controlsTable = solution.exportToControlsTable();
        TimeSeriesTable inputControlsTable = 
                solution.exportToInputControlsTable();
        const auto& controller = model.updComponent<SynergyController>(
                "/controllerset/synergy_controller");
        const auto& synergyVectors = controller.getSynergyVectorsAsMatrix();
        for (int i = 0; i < static_cast<int>(controlsTable.getNumRows()); ++i) {
            SimTK::Vector expected = 
                    synergyVectors * ~inputControlsTable.getRowAtIndex(i);
            SimTK_TEST_EQ(controlsTable.getRowAtIndex(i), expected.transpose());
        }
    }

    SECTION("Calling analyzeMocoTrajectory() is now valid") {
        CHECK_NOTHROW(
                analyzeMocoTrajectory<double>(model, solution, {"output"}));
    }
}

TEMPLATE_TEST_CASE("Triple pendulum with disabled InputController", "", 
        MocoCasADiSolver) {
    Model model = createControlledTriplePendulumModel(false);
    MocoStudy study = createTriplePendulumMocoStudy<TestType>(model);
    MocoSolution solution = study.solve();
    TimeSeriesTable expected = solution.exportToControlsTable();

    SECTION("Solution size is correct") {
        CHECK(solution.getNumInputControls() == 0);
        CHECK(solution.getNumControls() == 3);
    }

    SECTION("generateControlsFromModelControllers() should do nothing") {
        solution.generateControlsFromModelControllers(model);
        CHECK(solution.getNumControls() == 3);
        TimeSeriesTable actual = solution.exportToControlsTable();
        OpenSim_REQUIRE_MATRIX(actual.getMatrix(), expected.getMatrix());
    }
}

TEST_CASE("Controllers with disabled actuators") {
    Model model = createControlledTriplePendulumModel();
    model.updComponent<Actuator>("/forceset/tau0").set_appliesForce(false);
    std::string expected = "Expected all actuators controlled by "
            "'/controllerset/synergy_controller' to be enabled";
    CHECK_THROWS_WITH(createTriplePendulumMocoStudy<MocoCasADiSolver>(model), 
            ContainsSubstring(expected));
}

TEST_CASE("No actuators connected to controller") {
    Model model = createTriplePendulum();
    auto* controller = new SynergyController();
    controller->setName("synergy_controller");
    model.addController(controller);
    model.finalizeConnections();
    std::string expected = 
            "Controller '/controllerset/synergy_controller' "
            "has no actuators connected.";
    CHECK_THROWS_WITH(createTriplePendulumMocoStudy<MocoCasADiSolver>(model), 
            ContainsSubstring(expected));
}

TEST_CASE("Incorrect Input control info") {
    Model model = createControlledTriplePendulumModel();
    MocoStudy study = createTriplePendulumMocoStudy<MocoCasADiSolver>(model);
    auto& problem = study.updProblem();
    problem.setInputControlInfo("incorrect", {});
    auto& solver = study.updSolver<MocoCasADiSolver>();
    std::string expected = "Input control info provided for nonexistent "
            "Input control 'incorrect'.";
    CHECK_THROWS_WITH(solver.resetProblem(problem), 
            ContainsSubstring(expected));
}

TEMPLATE_TEST_CASE("MocoControlGoal: ignoring Input controls", "",
        MocoCasADiSolver) {
    Model model = createControlledTriplePendulumModel();
    // Must also ignore the controlled actuators to get an objective value of 0.
    bool ignoreControlledActuators = true;
    bool ignoreInputControls = true;
    MocoStudy study = createTriplePendulumMocoStudy<TestType>(model, 
            ignoreControlledActuators, ignoreInputControls);
    auto& solver = study.updSolver<TestType>();
    solver.set_optim_max_iterations(1);
    MocoSolution solution = study.solve().unseal();
    CHECK(solution.getObjective() == 0);
}

TEMPLATE_TEST_CASE("MocoControlBoundConstraint with Input controls", "",
        MocoCasADiSolver) {
    Model model = createControlledTriplePendulumModel();
    MocoStudy study = createTriplePendulumMocoStudy<TestType>(model);
    auto& problem = study.updProblem();
    auto* controlBoundPC = 
            problem.addPathConstraint<MocoControlBoundConstraint>();
    controlBoundPC->addControlPath(
            "/controllerset/synergy_controller/synergy_excitation_0");
    controlBoundPC->setLowerBound(Constant(-10));
    auto& solver = study.updSolver<TestType>();
    solver.resetProblem(problem);
    MocoSolution solution = study.solve();
    int numTimes = solution.getNumTimes();
    const auto& synergyControl0 = solution.getInputControl(
            "/controllerset/synergy_controller/synergy_excitation_0");
    for (int i = 0; i < numTimes; ++i) {
        CHECK(synergyControl0[i] >= Approx(-10).margin(1e-6));
    }
}

TEMPLATE_TEST_CASE("MocoControlTrackingGoal with Input controls", "",
        MocoCasADiSolver) {
    Model model = createControlledTriplePendulumModel();
    MocoStudy study = createTriplePendulumMocoStudy<TestType>(model);

    auto& problem = study.updProblem();
    problem.updGoal("effort").setEnabled(false);
    problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0);
    auto* controlTrackingGoal = problem.addGoal<MocoControlTrackingGoal>();
    controlTrackingGoal->setName("control_tracking");
    int N = 10;
    SimTK::Vector time = createVectorLinspace(10, 0, 0.5);
    std::vector<double> indVec;
    SimTK::Vector controlRef(N);
    for (int i = 0; i < N; ++i) {
        indVec.push_back(time[i]);
        controlRef[i] = -12.34;
    }
    TimeSeriesTable ref(indVec);
    ref.appendColumn(
            "/controllerset/synergy_controller/synergy_excitation_0", 
            controlRef);
    controlTrackingGoal->setReference(TableProcessor(ref));
    auto& solver = study.updSolver<TestType>();
    solver.resetProblem(problem);

    MocoSolution solution = study.solve();
    int numTimes = solution.getNumTimes();
    const auto& synergyControl0 = solution.getInputControl(
            "/controllerset/synergy_controller/synergy_excitation_0");
    for (int i = 0; i < numTimes; ++i) {
        CHECK(synergyControl0[i] == Approx(-12.34).margin(1e-6));
    }
}

TEST_CASE("MocoPeriodicityGoal with Input controls") {
    Model model = createControlledTriplePendulumModel();
    MocoStudy study = createTriplePendulumMocoStudy<MocoCasADiSolver>(model);
    auto& problem = study.updProblem();
    auto* periodicity = 
            problem.addGoal<MocoPeriodicityGoal>();
    periodicity->addControlPair(MocoPeriodicityGoalPair(
            "/controllerset/synergy_controller/synergy_excitation_0"));
    auto& solver = study.updSolver<MocoCasADiSolver>();
    solver.resetProblem(problem);
    MocoSolution solution = study.solve();
    int N = solution.getNumTimes();
    const auto& synergyControl0 = solution.getInputControl(
            "/controllerset/synergy_controller/synergy_excitation_0");
    CHECK(synergyControl0[0] == Approx(synergyControl0[N-1]).margin(1e-6));
}

TEST_CASE("SynergyController") {
    Model model = createTriplePendulum();
    auto* controller = new SynergyController();
    controller->setName("synergy_controller");
    controller->addActuator(
            model.getComponent<CoordinateActuator>("/forceset/tau0"));
    controller->addActuator(
            model.getComponent<CoordinateActuator>("/forceset/tau1"));
    controller->addActuator(
            model.getComponent<CoordinateActuator>("/forceset/tau2"));
    controller->addSynergyVector(createVector({0.25, 0.75, 0.5}));

    SECTION("Invalid synergy vector index") {
        model.addController(controller);
        model.finalizeConnections();
        CHECK_THROWS_WITH(controller->getSynergyVector(-1), 
                ContainsSubstring("Expected a non-negative synergy vector"));
        CHECK_THROWS_WITH(controller->updSynergyVector(-1, createVector({})), 
                ContainsSubstring("Expected a non-negative synergy vector"));
    }

    SECTION("Synergy vector index outside range") {
        model.addController(controller);
        model.finalizeConnections();
        CHECK_THROWS_WITH(controller->getSynergyVector(1), 
                ContainsSubstring("Expected a synergy vector index"));
        CHECK_THROWS_WITH(controller->updSynergyVector(1, createVector({})), 
                ContainsSubstring("Expected a synergy vector index"));
    }

    SECTION("Incorrect synergy vector size") {
        controller->addSynergyVector(createVector({0.5, 0.5}));
        model.addController(controller);
        REQUIRE_THROWS_WITH(model.finalizeConnections(),
                ContainsSubstring("Expected 'synergy_vector_1' to have size "));
    }
}
