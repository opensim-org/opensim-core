#ifndef MOCO_MOCOCONTROLCOST_H
#define MOCO_MOCOCONTROLCOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoControlCost.h                                            *
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

#include "MocoCost.h"
#include "../MocoWeightSet.h"

namespace OpenSim {

/// Minimize the sum of squared controls, integrated over the phase.
/// The default weight for each control is 1.0; this can be changed by
/// calling setWeight() or editing the `control_weights` property in XML.
/// @ingroup mococost
// TODO want a related cost for minimizing the value of state variables like
// activation.
// TODO allow leaving out some controls.
class OSIMMOCO_API MocoControlCost : public MocoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoControlCost, MocoCost);
public:
    MocoControlCost();
    MocoControlCost(std::string name) : MocoCost(std::move(name)) {
        constructProperties();
    }
    MocoControlCost(std::string name, double weight)
            : MocoCost(std::move(name), weight) {
        constructProperties();
    }
    /// Set the weight to use for the term in the cost associated with
    /// `controlName` (the name or path of the corresponding actuator). If a
    /// weight is already set for the requested state, then the provided
    /// weight replaces the previous weight.
    void setWeight(const std::string& controlName, const double& weight);
protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegralCostImpl(const SimTK::State& state,
            double& integrand) const override;
private:
    void constructProperties();
    OpenSim_DECLARE_PROPERTY(control_weights, MocoWeightSet,
            "The weights for each control; "
            "the weight for unspecified controls is 1.");
    mutable std::vector<double> m_weights;
};

} // namespace OpenSim

#endif // MOCO_MOCOCONTROLCOST_H
