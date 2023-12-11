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

#include "OpenSim/Actuators/CoordinateActuator.h"
#include "OpenSim/Actuators/ModelFactory.h"
#include "OpenSim/Simulation/Control/DiscreteController.h"
#include <OpenSim/Simulation/Control/InputController.h>

#include <OpenSim/Moco/osimMoco.h>

using namespace OpenSim;

class ControlAllocator : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(ControlAllocator, Component);
public:
    // LIST OUTPUT
    OpenSim_DECLARE_LIST_OUTPUT(
        controls, double, getControlForOutputChannel, SimTK::Stage::Dynamics);

    // CONSTRUCTION
    ControlAllocator() = default;
    ControlAllocator(const ControlAllocator&) = default;
    ControlAllocator(ControlAllocator&&) = default;
    ControlAllocator& operator=(const ControlAllocator&) = default;
    ControlAllocator& operator=(ControlAllocator&&) = default;

    // METHODS
    void addControl(const std::string& controlName) {
        const int index = m_controlIndexMap.size();
        m_controlIndexMap[controlName] = index;
    }

    void setControls(SimTK::State& s,
            const SimTK::Vector& controls) const {
        updControls(s) = controls;
    }

    SimTK::Vector& updControls(SimTK::State& s) const {
        const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();
        auto& dv = subSys.updDiscreteVariable(s, m_discreteVarIndex);
        auto& discreteControls =
            SimTK::Value<SimTK::Vector>::updDowncast(dv).upd();
        return discreteControls;
    }

    const SimTK::Vector& getControls(
            const SimTK::State& s) const {
        const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();
        auto& dv = subSys.getDiscreteVariable(s, m_discreteVarIndex);
        auto& discreteControls = SimTK::Value<SimTK::Vector>::downcast(dv).get();
        return discreteControls;
    }

    double getControlForOutputChannel(const SimTK::State& s,
            const std::string& channel) const {
        return getControls(s)[m_controlIndexMap.at(channel)];
    }

protected:
    void extendRealizeTopology(SimTK::State& state) const {
        Super::extendRealizeTopology(state);
        const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();

        const SimTK::Vector initControls(
            static_cast<int>(m_controlIndexMap.size()), 0.0);

        m_discreteVarIndex =
                subSys.allocateDiscreteVariable(state, SimTK::Stage::Dynamics,
                        new SimTK::Value<SimTK::Vector>(initControls));
    }


    void extendFinalizeFromProperties() {
        Super::extendFinalizeFromProperties();
        for (const auto& kv : m_controlIndexMap) {
            updOutput("controls").addChannel(kv.first);
        }
    }

private:
    std::unordered_map<std::string, int> m_controlIndexMap;
    mutable SimTK::DiscreteVariableIndex m_discreteVarIndex;

};

//

// class InputController : public Controller {
//     OpenSim_DECLARE_ABSTRACT_OBJECT(InputController, Controller);
// public:
//     OpenSim_DECLARE_LIST_INPUT(controls, double, SimTK::Stage::Dynamics, "controls");
//     InputController() = default;
// };
//
// class ActuatorController : public InputController {
//     OpenSim_DECLARE_CONCRETE_OBJECT(ActuatorController, InputController);
// public:
//     ActuatorController() = default;
//
//     void computeControls(const SimTK::State& s,
//             SimTK::Vector& controls) const {
//
//         SimTK::Vector control(1, 0.0);
//         for (int i = 0; i < getActuatorSet().getSize(); ++i) {
//             const auto& actu = getActuatorSet().get(i);
//             const unsigned index = m_aliasMap.at(actu.getName());
//             control[0] = getInput<double>("controls").getValue(s, index);
//             actu.addInControls(control, controls);
//         }
//
//     }
//
// protected:
//     void extendFinalizeFromProperties() {
//         Super::extendFinalizeFromProperties();
//         const auto& input = getInput<double>("controls");
//         for (int i = 0; i < input.getNumConnectees(); ++i) {
//             const auto& alias = input.getAlias(i);
//             OPENSIM_THROW_IF(!getActuatorSet().contains(alias), Exception,
//                 "Actuator '{}' not found in the ActuatorController's "
//                 "ActuatorSet.", alias);
//             m_aliasMap[alias] = i;
//         }
//     }
//
// private:
//     std::unordered_map<std::string, unsigned> m_aliasMap;
//
// };

int main() {

    Model model = ModelFactory::createDoublePendulum();
    model.initSystem();

    ControlAllocator* allocator = new ControlAllocator;
    allocator->setName("control_allocator");
    allocator->addControl("/tau0");
    allocator->addControl("/tau1");
    model.addComponent(allocator);

    ActuatorInputController* controller = new ActuatorInputController;
    controller->setName("actuator_controller");
    controller->addActuator(model.getComponent<CoordinateActuator>("/tau0"));
    controller->addActuator(model.getComponent<CoordinateActuator>("/tau1"));
    controller->connectInput_controls(
        allocator->getOutput("controls").getChannel("/tau0"), "/tau0");
    controller->connectInput_controls(
        allocator->getOutput("controls").getChannel("/tau1"), "/tau1");
    model.addController(controller);
    model.finalizeFromProperties();
    model.finalizeConnections();

    SimTK::State state = model.initSystem();

    SimTK::Vector newControls(2, 0.0);
    newControls[0] = 1.0;
    newControls[1] = 2.0;
    allocator->setControls(state, newControls);
    model.realizeDynamics(state);

    model.printSubcomponentInfo();

    std::cout << "actuation tau0: " << model.getComponent<CoordinateActuator>("/tau0").getActuation(state) << std::endl;
    std::cout << "actuation tau1: " << model.getComponent<CoordinateActuator>("/tau1").getActuation(state) << std::endl;

    return EXIT_SUCCESS;
}
