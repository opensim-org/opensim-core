/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoInitialForceEquilibriumDGFGoal.cpp                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
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

#include "MocoInitialForceEquilibriumDGFGoal.h"
#include "OpenSim/Actuators/DeGrooteFregly2016Muscle.h"

using namespace OpenSim;

void MocoInitialForceEquilibriumDGFGoal::initializeOnModelImpl(
        const Model& model) const {

    for (const auto& dgfmuscle :
            model.getComponentList<DeGrooteFregly2016Muscle>()) {
        if (!dgfmuscle.get_ignore_tendon_compliance() &&
                dgfmuscle.get_tendon_compliance_dynamics_mode() == "explicit") {
            m_muscleRefs.emplace_back(dgfmuscle);
        }
    }

    setRequirements(0, (int)m_muscleRefs.size());
}

void MocoInitialForceEquilibriumDGFGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& goal) const {
    const auto& s = input.initial_state;
    getModel().realizeVelocity(s);

    for (int i = 0; i < (int)m_muscleRefs.size(); ++i) {
        const auto muscleTendonLength = m_muscleRefs[i]->getLength(s);
        const auto muscleTendonVelocity =
                m_muscleRefs[i]->getLengtheningSpeed(s);
        const auto activation = m_muscleRefs[i]->getActivation(s);
        const auto normTendonForce =
                m_muscleRefs[i]->getNormalizedTendonForce(s);
        // Assuming normalized tendon force derivative is zero.
        const auto residual =
                m_muscleRefs[i]->calcEquilibriumResidual(muscleTendonLength,
                        muscleTendonVelocity, activation, normTendonForce, 0);
        goal[i] = residual;
        if (getModeIsCost()) { goal[i] *= goal[i]; }
    }
}