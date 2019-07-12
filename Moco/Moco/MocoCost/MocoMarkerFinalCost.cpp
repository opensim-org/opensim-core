/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoMarkerFinalCost.cpp                                      *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include "MocoMarkerFinalCost.h"

#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

void MocoMarkerFinalCost::initializeOnModelImpl(const Model& model) const {
    m_point.reset(&model.getComponent<Point>(get_point_name()));
}

void MocoMarkerFinalCost::calcCostImpl(
        const CostInput& input, SimTK::Real& cost) const {
    getModel().realizePosition(input.final_state);
    const auto& actualLocation = m_point->getLocationInGround(input.final_state);
    cost = (actualLocation - get_reference_location()).normSqr();
}

void MocoMarkerFinalCost::constructProperties() {
    constructProperty_point_name("");
    constructProperty_reference_location(SimTK::Vec3(0));
}
