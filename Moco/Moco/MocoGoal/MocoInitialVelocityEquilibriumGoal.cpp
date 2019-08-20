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

#include "MocoInitialVelocityEquilibriumGoal.h"

using namespace OpenSim;

void MocoInitialVelocityEquilibriumGoal::initializeOnModelImpl(
        const Model& model) const {

    for (const auto& dgfmuscle : 
            model.getComponentList<DeGrooteFregly2016Muscle>()) {
        if (!dgfmuscle.get_ignore_tendon_compliance()) {
            m_dgfMuscleRefs.emplace_back(&dgfmuscle);
        }
    }

    setNumIntegralsAndOutputs(0, (int)m_dgfMuscleRefs.size());
}

void MocoInitialVelocityEquilibriumGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& goal) const {
    const auto& s = input.initial_state;

    int i = 0;
    for (const auto& dgfMuscleRef : m_dgfMuscleRefs) {
        const auto& dgfmuscle = dgfMuscleRef.getRef();

        // Equation (A6) in Millard et al. 2013:
        //      residual = dFS_dlS * vS - dFT_dlT * (vMT - vS)
        goal[i] = dgfmuscle.calcDerivativeLinearizedEquilibriumResidual(s);
    }
}