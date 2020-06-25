#ifndef OPENSIM_MOCOUSERCONTROLCOST_H
#define OPENSIM_MOCOUSERCONTROLCOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoUserControlCost.h                                             *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Prasanna Sritharan, Christopher Dembia                          *
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

#include <Moco/MocoWeightSet.h>
#include <Moco/MocoCost/MocoCost.h>
#include <functional>
#include <vector>

namespace OpenSim {

/// Minimize the sum of squared controls, integrated over the phase.
/// The default weight for each control is 1.0; this can be changed by
/// calling setWeight() or editing the `control_weights` property in XML.
/// @ingroup mococost
// TODO want a related cost for minimizing the value of state variables like
// activation.
// TODO allow leaving out some controls.
class MocoUserControlCost : public MocoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoUserControlCost, MocoCost);

public:
    // Function reference for user defined control cost, ideally this should be
    // a property. Default: nullptr
    std::function<double(const SimTK::State&, const Model&, std::vector<double>,
            std::vector<double>, std::vector<int>)>
            user_control_cost_fun_ptr;

    // Vector of parameters for use in user-defined control cost function. This
    // is provided as a convenience. Unpack this vector within your user-defined
    // function to use the individual values,  ideally this should be a
    // property. Default: empty vector.
    std::vector<double> utility_vector;

    MocoUserControlCost();
    MocoUserControlCost(std::string name) : MocoCost(std::move(name)) {
        constructProperties();
    }
    MocoUserControlCost(std::string name, double weight)
            : MocoCost(std::move(name), weight) {
        constructProperties();
    }
    /// Set the weight to use for the term in the cost associated with
    /// `controlName` (the name or path of the corresponding actuator). To
    /// remove a control from the cost function, set its weight to 0. If a
    /// weight is already set for the requested state, then the provided
    /// weight replaces the previous weight. Only controls with non-zero weights
    /// that are associated with actuators for which appliesForce is True are
    /// included in the cost function.
    void setWeight(const std::string& controlName, const double& weight);

protected:
    void initializeOnModelImpl(const Model&) const override;
    int getNumIntegralsImpl() const override { return 1; }
    void calcIntegrandImpl(
            const SimTK::State& state, double& integrand) const override;

    void calcCostImpl(
            const CostInput& input, SimTK::Real& cost) const override {
        cost = input.integral;
    }

private:
    void constructProperties();
    OpenSim_DECLARE_PROPERTY(control_weights, MocoWeightSet,
            "The weights for each control; "
            "the weight for unspecified controls is 1.");
    mutable std::vector<double> m_weights;
    mutable std::vector<int> m_controlIndices;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOUSERCONTROLCOST_H
