/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MinettiAlexander1997Metabolics.cpp                           *
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

#include "MinettiAlexander1997Metabolics.h"

#include <SimTKcommon/internal/State.h>

#include <OpenSim/Common/Component.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

double MinettiAlexander1997Metabolics::getTotalMetabolicRate(
        const SimTK::State& s) const {
    return getMetabolicRate(s).sum();
}

double MinettiAlexander1997Metabolics::getMuscleMetabolicRate(
        const SimTK::State& s, const std::string& channel) const {
    return getMetabolicRate(s).get(m_muscleIndices.at(channel));
}

void MinettiAlexander1997Metabolics::extendConnectToModel(Model& model) {
    // TODO: Should this be in extendFinalizeFromProperties()?
    m_muscles.clear();
    m_muscleIndices.clear();
    int i = 0;
    for (const auto& muscle : model.getComponentList<Muscle>()) {
        if (muscle.get_appliesForce()) {
            // This "maxPower" is Fmax * Vmax.
            // Must convert max contraction velocity from
            // optimal_fiber_lengths/second to meters/second.
            const double maxPower = muscle.get_max_isometric_force() *
                                    muscle.get_max_contraction_velocity() *
                                    muscle.get_optimal_fiber_length();
            m_muscles.emplace_back(&muscle, maxPower);
            m_muscleIndices[muscle.getAbsolutePathString()] = i;
            ++i;
        }
    }
}

void MinettiAlexander1997Metabolics::extendAddToSystem(
        SimTK::MultibodySystem&) const {
    addCacheVariable<SimTK::Vector>("metabolic_rate",
            SimTK::Vector((int)m_muscles.size(), 0.0), SimTK::Stage::Velocity);
}

const SimTK::Vector& MinettiAlexander1997Metabolics::getMetabolicRate(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, "metabolic_rate")) {
        calcMetabolicRate(
                s, updCacheVariableValue<SimTK::Vector>(s, "metabolic_rate"));
        markCacheVariableValid(s, "metabolic_rate");
    }
    return getCacheVariableValue<SimTK::Vector>(s, "metabolic_rate");
}

void MinettiAlexander1997Metabolics::calcMetabolicRate(
        const SimTK::State& s, SimTK::Vector& ratesForMuscles) const {
    ratesForMuscles.resize((int)m_muscles.size());
    int i = 0;
    for (const auto& entry : m_muscles) {
        const double activation = entry.first->getActivation(s);
        const double& maxPower = entry.second;
        const double normFiberShortVel =
                -entry.first->getNormalizedFiberVelocity(s);

        const double numerator =
                0.054 * normFiberShortVel * (0.506 + normFiberShortVel * 2.46);
        const double denominator =
                1 +
                normFiberShortVel *
                        (-1.13 + normFiberShortVel *
                                         (12.8 + normFiberShortVel * -1.64));

        ratesForMuscles[i] = activation * maxPower * numerator / denominator;
        ++i;
    }
}
