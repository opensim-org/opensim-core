#ifndef MOCO_MOCOSUMSQUAREDSTATECOST_H
#define MOCO_MOCOSUMSQUAREDSTATECOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoSumSquaredStateCost.h                                    *
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

#include "MocoCost.h"

namespace OpenSim {

/// Minimize the sum of squared states, integrated over the phase. This
/// can be used to minimize muscle activations (if those are the only states
/// in the system).
/// @underdevelopment
/// In the future, this class will allow you to select which states to
/// minimize.
class OSIMMOCO_API MocoSumSquaredStateCost : public MocoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoSumSquaredStateCost, MocoCost);

public:
    MocoSumSquaredStateCost();
    MocoSumSquaredStateCost(std::string name) : MocoCost(std::move(name)) {
        constructProperties();
    }
    MocoSumSquaredStateCost(std::string name, double weight)
            : MocoCost(std::move(name), weight) {
        constructProperties();
    }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegralCostImpl(
            const SimTK::State& state, double& integrand) const override;

private:
    void constructProperties();
};

} // namespace OpenSim

#endif // MOCO_MOCOSUMSQUAREDSTATECOST_H
