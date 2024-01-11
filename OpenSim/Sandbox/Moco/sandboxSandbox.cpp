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

#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Moco/osimMoco.h>

using namespace OpenSim;

class TestController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(TestController, Controller);
public:
    void computeControls(const SimTK::State& s,
            SimTK::Vector& controls) const override {
        log_cout("Computing controls...");
    }

    void setActuatorPath(const std::string& path) {
        m_actuatorPath = path;
    }

    const std::string& getActuatorPath() const {
        return m_actuatorPath;
    }

protected:
    void extendConnectToModel(Model& model) override {
        const auto& socket = getSocket<Actuator>("actuators");
        const auto& actuPath = getActuatorPath();
        int index = socket.getConnecteePathIndex(actuPath);
        log_cout("extendConnectToModel: connectee path index = {}", index);

    }

    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        const auto& socket = getSocket<Actuator>("actuators");
        const auto& actuPath = getActuatorPath();
        int index = socket.getConnecteePathIndex(actuPath);
        log_cout("extendAddToSystem: connectee path index = {}", index);
    }

    void extendFinalizeFromProperties() override {

        const auto& socket = getSocket<Actuator>("actuators");
        const auto& actuPath = getActuatorPath();
        int index = socket.getConnecteePathIndex(actuPath);
        log_cout("extendAddToSystem: connectee path index = {}", index);

    }

private:
    std::string m_actuatorPath;

};

int main() {

    Model model = ModelFactory::createNLinkPendulum(5);

    auto* controller = new TestController();
    controller->setActuatorPath("/tau3");
    controller->appendConnectee_actuators(model.getComponent<Actuator>("tau0"));
    controller->appendConnectee_actuators(model.getComponent<Actuator>("tau1"));
    controller->appendConnectee_actuators(model.getComponent<Actuator>("tau2"));
    controller->appendConnectee_actuators(model.getComponent<Actuator>("tau3"));
    controller->appendConnectee_actuators(model.getComponent<Actuator>("tau4"));

    model.addController(controller);

    model.initSystem();

    return EXIT_SUCCESS;
}
