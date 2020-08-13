/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testMocoActuators.cpp                                        *
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

// Some of this code is based on testSingleMuscle,
// testSingleMuscleDeGrooteFregly2016.

#include <OpenSim/Actuators/ActivationCoordinateActuator.h>
#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/CMCTool.h>

#define CATCH_CONFIG_MAIN
#include "Testing.h"

using namespace OpenSim;

TEST_CASE("ActivationCoordinateActuator") {
    // Create a problem with ACA and ensure the activation bounds are
    // set as expected.
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
    CHECK(rep.getStateInfo("/forceset/aca/activation").getBounds().getLower() ==
            Approx(-0.31));
    CHECK(rep.getStateInfo("/forceset/aca/activation").getBounds().getUpper() ==
            Approx(0.78));
}
