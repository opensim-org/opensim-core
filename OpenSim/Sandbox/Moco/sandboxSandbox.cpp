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
        std::vector<double> weights = {0.25, 0.75};

        SimTK::Vector actControls(1, 0.0);
        const auto& socket = getSocket<Actuator>("actuators");
        for(int i = 0; i < (int)socket.getNumConnectees(); ++i){
            actControls[0] = weights[i]*synergyControl;
            socket.getConnectee(i).addInControls(actControls, controls);
        }
    }
};

int main() {

    Model model = ModelFactory::createDoublePendulum();
    // auto* controller = new DoublePendulumController();
    // controller->setName("double_pendulum_controller");
    // controller->addActuator(model.getComponent<CoordinateActuator>("/tau0"));
    // controller->addActuator(model.getComponent<CoordinateActuator>("/tau1"));
    // model.addController(controller);
    // model.finalizeConnections();

    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelAsCopy(model);
    problem.setTimeBounds(0, 1);
    problem.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0);
    problem.setStateInfo("/jointset/j0/q0/speed", {-50, 50}, 0);
    problem.setStateInfo("/jointset/j1/q1/value", {-10, 10}, 0);
    problem.setStateInfo("/jointset/j1/q1/speed", {-50, 50}, 0);
    problem.setControlInfo("/tau0", {-100, 100});
    problem.setControlInfo("/tau1", {-50, 50});
    // problem.setInputControlInfo(
    //     "/controllerset/double_pendulum_controller/synergy_control", 
    //         {-500, 500});
    problem.addGoal<MocoControlGoal>();
    auto& solver = study.initCasADiSolver();
    solver.set_parallel(0);

    study.solve();

    return EXIT_SUCCESS;
}
