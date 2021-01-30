/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoInitialVelocityEquilibriumGoal.cpp                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

#include "MocoInitialVelocityEquilibriumDGFGoal.h"

#include <OpenSim/Actuators/DeGrooteFregly2016Muscle.h>

using namespace OpenSim;

void MocoInitialVelocityEquilibriumDGFGoal::initializeOnModelImpl(
        const Model& model) const {

    for (const auto& dgfmuscle :
            model.getComponentList<DeGrooteFregly2016Muscle>()) {
        if (!dgfmuscle.get_ignore_tendon_compliance()) {
            m_dgfMuscleRefs.emplace_back(&dgfmuscle);
        }
    }

    setRequirements(0, (int)m_dgfMuscleRefs.size());
}

void MocoInitialVelocityEquilibriumDGFGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& goal) const {
    const auto& s = input.initial_state;
    getModel().realizeVelocity(s);
    if (getModeIsCost()) {
        for (int i = 0; i < (int)m_dgfMuscleRefs.size(); ++i) {
            const auto residual =
                    m_dgfMuscleRefs[i]
                            ->getLinearizedEquilibriumResidualDerivative(s);
            goal[i] = residual * residual;
        }
    } else {
        for (int i = 0; i < (int)m_dgfMuscleRefs.size(); ++i) {
            const auto residual =
                    m_dgfMuscleRefs[i]
                            ->getLinearizedEquilibriumResidualDerivative(s);
            goal[i] = residual;
        }
    }
}
