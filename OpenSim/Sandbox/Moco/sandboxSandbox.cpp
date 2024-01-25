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
#include <OpenSim/Simulation/Control/InputController.h>
#include <OpenSim/Moco/Components/ControlAllocator.h>

#include <OpenSim/Moco/osimMoco.h>

using namespace OpenSim;


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
    newControls[0] = 1.23;
    newControls[1] = 4.56;
    allocator->setControls(state, newControls);
    model.realizeDynamics(state);

    model.printSubcomponentInfo();

    std::cout << "actuation tau0: " << model.getComponent<CoordinateActuator>("/tau0").getActuation(state) << std::endl;
    std::cout << "actuation tau1: " << model.getComponent<CoordinateActuator>("/tau1").getActuation(state) << std::endl;

    return EXIT_SUCCESS;
}
