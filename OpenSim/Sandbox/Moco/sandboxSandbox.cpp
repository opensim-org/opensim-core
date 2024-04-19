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
    OpenSim_DECLARE_CONCRETE_OBJECT(DoublePendulumController, InputController);

public:
    std::vector<std::string> getInputControlLabels() const override {
        return {"synergy_control"};
    }

    void computeControlsImpl(const SimTK::State& state,
            SimTK::Vector& controls) const override {
        const auto& input = getInput<double>("controls");
        double synergyControl = input.getValue(state, 0);
        std::vector<double> weights = {0.25, 0.75};

        SimTK::Vector actControls(1, 0.0);
        const auto& socket = getSocket<Actuator>("actuators");
        for(int i = 0; i < (int)socket.getNumConnectees(); ++i){
            actControls[0] = weights[i]*synergyControl;
            socket.getConnectee(i).addInControls(actControls, controls);
        }
    }
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
    controller->setName("double_pendulum_controller");
    controller->addActuator(model.getComponent<CoordinateActuator>("/tau0"));
    controller->addActuator(model.getComponent<CoordinateActuator>("/tau1"));
    model.addController(controller);
    model.finalizeConnections();

    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelAsCopy(model);
    problem.setTimeBounds(0, 1);
    problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0);
    problem.setStateInfo("/jointset/j0/q0/speed", {-50, 50}, 0);
    problem.setStateInfo("/jointset/j1/q1/value", {-10, 10}, 0);
    problem.setStateInfo("/jointset/j1/q1/speed", {-50, 50}, 0);
    problem.setInputControlInfo(
        "/controllerset/double_pendulum_controller/synergy_control", 
            {-1, 1});
    auto* effort = problem.addGoal<MocoControlGoal>();
    effort->setIgnoreControlledActuators(false);
    effort->setIgnoreInputControls(true);
    auto& solver = study.initCasADiSolver();
    solver.set_parallel(0);

    study.solve();

}
int main() {
    testDoublePendulum();
    // testSlidingMass();
    return EXIT_SUCCESS;
}
