#ifndef MOCO_MINETTIALEXANDER1997METABOLICS_H
#define MOCO_MINETTIALEXANDER1997METABOLICS_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MinettiAlexander1997Metabolics.                              *
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

#include "../osimMocoDLL.h"
#include <unordered_map>

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/Muscle.h>

namespace OpenSim {

/// https://doi.org/10.1371/journal.pone.0222037
class OSIMMOCO_API MinettiAlexander1997Metabolics : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MinettiAlexander1997Metabolics, ModelComponent);

public:
    OpenSim_DECLARE_OUTPUT(total_metabolic_rate, double, getTotalMetabolicRate,
            SimTK::Stage::Velocity);
    OpenSim_DECLARE_LIST_OUTPUT(muscle_metabolic_rate, double,
            getMuscleMetabolicRate, SimTK::Stage::Velocity);

    double getTotalMetabolicRate(const SimTK::State& s) const;
    double getMuscleMetabolicRate(
            const SimTK::State& s, const std::string& channel) const;

private:
    void extendConnectToModel(Model&) override;
    void extendAddToSystem(SimTK::MultibodySystem&) const override;
    const SimTK::Vector& getMetabolicRate(const SimTK::State& s) const;
    void calcMetabolicRate(const SimTK::State& s, SimTK::Vector&) const;

    mutable std::vector<std::pair<SimTK::ReferencePtr<const Muscle>, double>>
            m_muscles;
    mutable std::unordered_map<std::string, int> m_muscleIndices;
};

} // namespace OpenSim

#endif // MOCO_MINETTIALEXANDER1997Metabolics_H
