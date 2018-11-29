/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoJointReactionNormCost.h                              *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include "MucoJointReactionNormCost.h"
#include <OpenSim/Simulation/Model/Model.h>
    
using namespace OpenSim;

MucoJointReactionNormCost::MucoJointReactionNormCost() {
    constructProperties();
}

void MucoJointReactionNormCost::constructProperties() {
    constructProperty_joint_path("");
}

void MucoJointReactionNormCost::initializeOnModelImpl(
        const Model& model) const {

    OPENSIM_THROW_IF_FRMOBJ(get_joint_path().empty(), Exception,
        "Empty model joint path detected. Please provide a valid joint path.");

    OPENSIM_THROW_IF_FRMOBJ(!model.hasComponent<Joint>(get_joint_path()),
        Exception, "Joint at path " + get_joint_path() + " not found in the "
        "model. Please provide a valid joint path.");
}

void MucoJointReactionNormCost::calcIntegralCostImpl(const SimTK::State& state,
        double& integrand) const {

    getModel().realizeAcceleration(state);
    // TODO: Cache the joint.
    const auto& joint = getModel().getComponent<Joint>(get_joint_path());
    integrand = joint.calcReactionOnChildExpressedInGround(state).norm();
}
