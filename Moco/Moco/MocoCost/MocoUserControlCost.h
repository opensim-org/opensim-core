#ifndef MOCO_MOCOUSERCONTROLCOST_H
#define MOCO_MOCOUSERCONTROLCOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoUserControlCost.h                                        *
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

#include "../MocoWeightSet.h"
#include "MocoCost.h"
#include <vector>
#include <functional>

namespace OpenSim {

/// Minimize the sum of squared controls, integrated over the phase.
/// The default weight for each control is 1.0; this can be changed by
/// calling setWeight() or editing the `control_weights` property in XML.
/// @ingroup mococost
// TODO want a related cost for minimizing the value of state variables like
// activation.
// TODO allow leaving out some controls.
class OSIMMOCO_API MocoUserControlCost : public MocoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoUserControlCost, MocoCost);

public:
    
	/// Functor that points to the user-defined control cost function. The functional 
	/// description must match the definition here, however, not all input parameters 
	/// need to be used.
	OpenSim_DECLARE_PROPERTY(user_control_cost_fun_ptr,
            std::function<double(const SimTK::State&, const Model&,
                    std::vector<double>, std::vector<double>,
                    std::vector<int>)>,
            "Functor representing the user-defined control cost function. "
            "Default: nullptr.");
    
	/// An optional utility vector to pass additional parameters into the user-defined 
	/// control cost function. Unpack this vector within your user-defined function to 
	/// utilise the individual values as required.
    OpenSim_DECLARE_PROPERTY(utility_vector, std::vector<double>,
            "Vector of parameters for use in user-defined control cost "
            "function. This is provided as a convenience. Unpack this vector "
            "within your user-defined function to use the individual values. "
            "Default: empty.");


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

#endif // MOCO_MOCOUSERCONTROLCOST_H
