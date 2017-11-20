#ifndef MUSCOLLO_MUCOCONTROLCOST_H
#define MUSCOLLO_MUCOCONTROLCOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoControlCost.h                                        *
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

#include "MucoCost.h"
#include "MucoWeightSet.h"

namespace OpenSim {

/// Minimize the sum of squared controls, integrated over the phase.
/// The default weight for each control is 1.0; this can be changed by
/// calling setWeight() or editing the `control_weights` property in XML.
// TODO want a related cost for minimizing the value of state variables like
// activation.
// TODO allow leaving out some controls.
class OSIMMUSCOLLO_API MucoControlCost : public MucoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoControlCost, MucoCost);
public:
    MucoControlCost();
    /// Set the weight to use for the term in the cost associated with
    /// `controlName` (the name or path of the corresponding actuator). If a
    /// weight is already set for the requested state, then the provided
    /// weight replaces the previous weight.
    void setWeight(const std::string& controlName, const double& weight);
protected:
    void initializeImpl() const override;
    void calcIntegralCostImpl(const SimTK::State& state,
            double& integrand) const override;
private:
    void constructProperties();
    OpenSim_DECLARE_PROPERTY(control_weights, MucoWeightSet,
            "The weights for each control; "
            "the weight for unspecified controls is 1.");
    mutable std::vector<double> m_weights;
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOCONTROLCOST_H
