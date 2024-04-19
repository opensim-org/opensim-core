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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Control/InputController.h>
#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Actuators/CoordinateActuator.h>

#include <catch2/catch_all.hpp>
#include "Testing.h"

using namespace OpenSim;

using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

TEMPLATE_TEST_CASE("Sliding mass with PrescribedController", "",
        MocoCasADiSolver, MocoTropterSolver) {

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
        solution.write("testMocoInterface_testSlidingMass_solution.sto");
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
            new GCVSpline(5, control.size(), time.data(), &control[0],
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
        solution.write(
                "testMocoInterface_testSlidingMass_prescribed_solution.sto");

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
            new Constant(1.0));
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

class TriplePendulumController : public InputController {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            TriplePendulumController, InputController);

public:
    TriplePendulumController() {
        m_synergyVectors.resize(3, 2);
        m_synergyVectors(0, 0) = 0.25;
        m_synergyVectors(0, 1) = 0.5;
        m_synergyVectors(1, 0) = 0.75;
        m_synergyVectors(1, 1) = 0.5;
        m_synergyVectors(2, 0) = 0.5;
        m_synergyVectors(2, 1) = 0.25;
    }

    std::vector<std::string> getInputControlLabels() const override {
        return {"synergy_control_0", "synergy_control_1"};
    }

    void computeControlsImpl(const SimTK::State& state,
            SimTK::Vector& controls) const override {
        const auto& input = getInput<double>("controls");
        for (int i = 0; i < input.getNumConnectees(); ++i) {
            controls += m_synergyVectors.col(i) * input.getValue(state, i);
        }

    }

    const SimTK::Matrix& getSynergyVectors() const { return m_synergyVectors; }

private:
    SimTK::Matrix m_synergyVectors;
};

TEST_CASE("InputController behavior") {
    Model model = ModelFactory::createNLinkPendulum(3);
    auto* controller = new TriplePendulumController();
    controller->setName("controller");
    controller->addActuator(model.getComponent<CoordinateActuator>("/tau0"));
    controller->addActuator(model.getComponent<CoordinateActuator>("/tau1"));
    controller->addActuator(model.getComponent<CoordinateActuator>("/tau2"));
    model.addComponent(controller);
    model.finalizeConnections();    

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

        auto& controller = 
                model.updComponent<TriplePendulumController>("/controller");
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

        auto& controller = 
                model.updComponent<TriplePendulumController>("/controller");
        controller.connectInput_controls(constant0->getOutput("signal"));
        controller.connectInput_controls(constant1->getOutput("signal"));
        model.finalizeConnections();
        SimTK::State state = model.initSystem();
        model.realizeVelocity(state);

        SimTK::Vector expected = controller.getSynergyVectors() * constants;
        SimTK::Vector controls = model.getControls(state);
        SimTK_TEST_EQ(expected, controls);
    }
}

TEST_CASE("Double pendulum with synergy-like InputController") {

    Model model = ModelFactory::createNLinkPendulum(3);
    auto* controller = new TriplePendulumController();
    controller->setName("controller");
    controller->addActuator(model.getComponent<CoordinateActuator>("/tau0"));
    controller->addActuator(model.getComponent<CoordinateActuator>("/tau1"));
    controller->addActuator(model.getComponent<CoordinateActuator>("/tau2"));
    model.addComponent(controller);
    model.finalizeConnections();

    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelAsCopy(model);
    problem.setTimeBounds(0, 1);
    problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0, 0.5*SimTK::Pi);
    problem.setStateInfo("/jointset/j0/q0/speed", {-50, 50}, 0, 0);
    problem.setStateInfo("/jointset/j1/q1/value", {-10, 10}, 0);
    problem.setStateInfo("/jointset/j1/q1/speed", {-50, 50}, 0);
    problem.setStateInfo("/jointset/j2/q2/value", {-10, 10}, 0);
    problem.setStateInfo("/jointset/j2/q2/speed", {-50, 50}, 0);
    problem.setInputControlInfo("/controller/synergy_control_0", {-1, 1});
    problem.setInputControlInfo("/controller/synergy_control_1", {-1, 1});
    problem.addGoal<MocoControlGoal>();
    auto& solver = study.initCasADiSolver();
    study.solve();    
}


// TODO things to test:
// - Check if no actuators are added to the controller.
// - Check if the controller has the wrong number of actuators.
// - Test when all controls are from ActuatorInputController.
// - Test when all controls are not from ActuatorInputController.
// - Test reusing solution with controllers as initial guess.