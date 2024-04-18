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

class DoublePendulumInputController : public InputController {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            DoublePendulumInputController, InputController);

public:
    std::vector<std::string> getExpectedInputChannelAliases() const override {
        return {"synergy_control"};
    }

    void checkInputConnections() const override {
        const auto& input = getInput<double>("inputs");
        OPENSIM_THROW_IF(static_cast<int>(input.getNumConnectees()) != 1,
            Exception, "Expected one input control connectee but received {}.",
            input.getNumConnectees());
    }

    void computeControls(const SimTK::State& state,
            SimTK::Vector& controls) const override {
        const auto& input = getInput<double>("inputs");
        double synergyControl = input.getValue(state, 0);
        controls[0] = 0.25*synergyControl;
        controls[1] = 0.75*synergyControl;
    }
};

TEST_CASE("Double pendulum synergy-like InputController") {

    Model model = ModelFactory::createDoublePendulum();
    auto* controller = new DoublePendulumInputController();
    controller->setName("double_pendulum_input_controller");
    controller->addActuator(model.getComponent<CoordinateActuator>("/tau0"));
    controller->addActuator(model.getComponent<CoordinateActuator>("/tau1"));
    model.addController(controller);
    model.finalizeConnections();

    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelAsCopy(model);
    problem.setTimeBounds(0, 1);
    problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0, 0.5*SimTK::Pi);
    problem.setStateInfo("/jointset/j0/q0/speed", {-50, 50}, 0, 0);
    problem.setStateInfo("/jointset/j1/q1/value", {-10, 10}, 0);
    problem.setStateInfo("/jointset/j1/q1/speed", {-50, 50}, 0);
    problem.setInputControlInfo(
        "/controllerset/double_pendulum_controller/synergy_control", {-1, 1});
    problem.addGoal<MocoControlGoal>();
    auto& solver = study.initCasADiSolver();
    study.solve();    
}


// TODO things to test:
// - Check if no actuators are added to the controller.
// - Check if the controller has the wrong number of actuators.
// - Test when all controls are from ActuatorInputController.
// - Test when all controls are not from ActuatorInputController.
// - Test that actuator Input connections are valid/invalid:
//       SECTION("Inputs not connected") {
//          CHECK_THROWS_AS(controller->checkInputConnections(), Exception);
//       }
// - Test reusing solution with controllers as initial guess.