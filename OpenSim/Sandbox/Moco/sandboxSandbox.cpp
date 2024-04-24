/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxSandbox.cpp                                           *
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

// This file provides a way to easily prototype or test temporary snippets of
// code during development.

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Simulation/Control/InputController.h>
#include <OpenSim/Actuators/CoordinateActuator.h>

using namespace OpenSim;

class DoublePendulumController : public InputController {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            DoublePendulumController, InputController);

public:
    DoublePendulumController() = default;

    std::vector<std::string> getInputControlLabels() const override {
        return {"synergy_control"};
    }

    void computeControlsImpl(const SimTK::State& state,
            SimTK::Vector& controls) const override {
        const auto& input = getInput<double>("controls");
        const auto& synergyControl = input.getValue(state, 0);
        
        controls[0] += 0.9 * synergyControl;
        controls[1] += 0.1 * synergyControl;
    }
};

class TriplePendulumController : public InputController {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            TriplePendulumController, InputController);

public:
    TriplePendulumController() {
        m_synergyVectors.resize(3, 2);
        m_synergyVectors(0, 0) = 1.0;
        m_synergyVectors(0, 1) = 0.0;
        m_synergyVectors(1, 0) = 0.0;
        m_synergyVectors(1, 1) = 0.25;
        m_synergyVectors(2, 0) = 0.0;
        m_synergyVectors(2, 1) = 0.75;
    }

    std::vector<std::string> getInputControlLabels() const override {
        return {"synergy_control_0", "synergy_control_1"};
    }

    void computeControlsImpl(const SimTK::State& state,
            SimTK::Vector& controls) const override {
        const auto& input = getInput<double>("controls");
        const auto& synergyControl0 = input.getValue(state, 0);
        const auto& synergyControl1 = input.getValue(state, 1);
        controls[0] = synergyControl0 * m_synergyVectors(0, 0) +
                synergyControl1 * m_synergyVectors(0, 1);
        controls[1] = synergyControl0 * m_synergyVectors(1, 0) +
                synergyControl1 * m_synergyVectors(1, 1);
        controls[2] = synergyControl0 * m_synergyVectors(2, 0) +
                synergyControl1 * m_synergyVectors(2, 1);
    }

    const SimTK::Matrix& getSynergyVectors() const { return m_synergyVectors; }

private:
    SimTK::Matrix m_synergyVectors;
};

void testSlidingMass() {

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
        auto& solver = study.initSolver<MocoCasADiSolver>();
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
        auto& solver = study.initSolver<MocoCasADiSolver>();
        solver.set_num_mesh_intervals(50);
        MocoSolution solution = study.solve();
        solution.write(
                "testMocoInterface_testSlidingMass_prescribed_solution.sto");

        // OpenSim_REQUIRE_MATRIX_ABSTOL(solution.getStatesTrajectory(),
        //     statesTrajectory, 1e-9);

        // // We should get back exactly the same controls trajectory that we
        // // provided via the PrescribedController.
        // OpenSim_REQUIRE_MATRIX_ABSTOL(solution.getControlsTrajectory(),
        //         controlsTable.getMatrix(), 1e-12);
        // REQUIRE(solution.getControlNames() == controlsTable.getColumnLabels());
    }
}

void testDoublePendulum() {

    Model model = ModelFactory::createDoublePendulum();
    auto* controller = new DoublePendulumController();
    controller->setName("controller");
    controller->addActuator(model.getComponent<CoordinateActuator>("/tau0"));
    controller->addActuator(model.getComponent<CoordinateActuator>("/tau1"));
    model.addComponent(controller);
    model.finalizeConnections();

    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelAsCopy(model);
    problem.setTimeBounds(0, 1);
    problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0);
    problem.setStateInfo("/jointset/j0/q0/speed", {-50, 50}, 0);
    problem.setStateInfo("/jointset/j1/q1/value", {-10, 10}, 0);
    problem.setStateInfo("/jointset/j1/q1/speed", {-50, 50}, 0);
    problem.setInputControlInfo("/controller/synergy_control", {-100, 100});
    auto* effort = problem.addGoal<MocoControlGoal>();
    effort->setIgnoreInputControls(true);
    effort->setIgnoreControlledActuators(true);
    auto& solver = study.initCasADiSolver();
    solver.set_parallel(0);
    MocoSolution solution = study.solve().unseal();

    // model.initSystem();
    // solution.insertControlsTrajectoryFromModel(problem.createRep());

    solution.write("sandbox_testDoublePendulum_solution.sto");

    std::cout << "num controls: " << solution.getNumControls() << std::endl;
    std::cout << "num input controls: " << solution.getNumInputControls() << std::endl;

    study.visualize(solution);

}

void testTriplePendulum() {

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
    problem.setTimeBounds(0, 2);
    problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0, 0.1*SimTK::Pi);
    problem.setStateInfo("/jointset/j0/q0/speed", {-50, 50}, 0);
    problem.setStateInfo("/jointset/j1/q1/value", {-10, 10}, 0);
    problem.setStateInfo("/jointset/j1/q1/speed", {-50, 50}, 0);
    problem.setStateInfo("/jointset/j2/q2/value", {-10, 10}, 0);
    problem.setStateInfo("/jointset/j2/q2/speed", {-50, 50}, 0);
    problem.setInputControlInfo("/controller/synergy_control_0", {-100, 100});
    problem.setInputControlInfo("/controller/synergy_control_1", {-100, 100});
    auto* effort = problem.addGoal<MocoControlGoal>();
    effort->setIgnoreInputControls(false);
    effort->setIgnoreControlledActuators(false);
    auto& solver = study.initCasADiSolver();
    MocoSolution solution = study.solve().unseal();

    solution.write("sandbox_testTriplePendulum_solution.sto");

    std::cout << "num controls: " << solution.getNumControls() << std::endl;
    std::cout << "num input controls: " << solution.getNumInputControls() << std::endl;

}

int main() {
    testDoublePendulum();
    // testSlidingMass();
    return EXIT_SUCCESS;
}
