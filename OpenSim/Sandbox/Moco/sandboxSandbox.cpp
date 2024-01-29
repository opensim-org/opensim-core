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

#include "OpenSim/Actuators/ActivationCoordinateActuator.h"
#include "OpenSim/Actuators/CoordinateActuator.h"
#include "OpenSim/Actuators/ModelFactory.h"

#include <OpenSim/Moco/Components/ControlAllocator.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/Control/InputController.h>

using namespace OpenSim;


int main() {
    auto model = ModelFactory::createSlidingPointMass();
    auto* actu = new ActivationCoordinateActuator();
    actu->setName("aca");
    actu->setCoordinate(&model.updCoordinateSet().get("position"));
    actu->setMinControl(-0.31);
    actu->setMaxControl(0.78);
    model.addForce(actu);
    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelAsCopy(model);
    auto rep = problem.createRep();


    return EXIT_SUCCESS;
}
