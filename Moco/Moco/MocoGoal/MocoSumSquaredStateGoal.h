#ifndef MOCO_MOCOSUMSQUAREDSTATEGOAL_H
#define MOCO_MOCOSUMSQUAREDSTATEGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoSumSquaredStateGoal.h                                    *
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

#include "MocoGoal.h"
#include "../MocoWeightSet.h"

namespace OpenSim {

/// Minimize the sum of squared states, integrated over the phase. This
/// can be used to minimize muscle activations (if those are the only states
/// in the system), as is done in MocoInverse.
/// @underdevelopment
/// In the future, this class will allow you to select which states to
/// minimize.
class OSIMMOCO_API MocoSumSquaredStateGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoSumSquaredStateGoal, MocoGoal);

public:
    MocoSumSquaredStateGoal();
    MocoSumSquaredStateGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoSumSquaredStateGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    /// Provide a MocoWeightSet to weight the state variables in the cost.
    /// Replaces the weight set if it already exists.
    void setWeightSet(const MocoWeightSet& weightSet) {
        upd_state_weights() = weightSet;
    }

    /// Set the weight for an individual state variable. If a weight is
    /// already set for the requested state, then the provided weight
    /// replaces the previous weight. An exception is thrown if a weight
    /// for an unknown state is provided.
    void setWeightForState(const std::string& stateName, const double& weight) {
        if (get_state_weights().contains(stateName)) {
            upd_state_weights().get(stateName).setWeight(weight);
        } else {
            upd_state_weights().cloneAndAppend({stateName, weight});
        }
    }

    /// Only state paths matching the regular expression are tracked. The
    /// regular expression must match the entire state path for a state path to
    /// be tracked (that is, we use std::regex_match, not std::regex_search).
    /// To track only generalized coordinates, use `.*value$`.
    /// To track generalized coordinates and speeds, use `.*(value|speed)$`.
    /// To track only activations, use `.*activation$`.
    /// If the reference contains columns for states whose path does not match
    /// this pattern, you will get an error unless you use
    /// `setAllowUnusedReferences(true)`.
    void setPattern(std::string pattern) { set_pattern(pattern); }
    /// Unset the pattern, which causes all states to be matched.
    void clearPattern() { updProperty_pattern().clear(); }
    std::string getPattern() const { return get_pattern(); }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const SimTK::State& state, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.integral;
    }
    void printDescriptionImpl(std::ostream& stream = std::cout) const override;

private:
    OpenSim_DECLARE_PROPERTY(state_weights, MocoWeightSet,
            "Set of weight objects to weight the tracking of individual "
            "state variables in the cost.");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(pattern, std::string,
            "If provided, only states matching this regular expression are "
            "tracked (default: no pattern). If no pattern is provided, then "
            "all states are used.");

    void constructProperties() {
        constructProperty_state_weights(MocoWeightSet());
        constructProperty_pattern();
    }

    /// The indices in Y corresponding to the provided reference coordinates.
    mutable std::vector<int> m_sysYIndices;
    mutable std::vector<double> m_state_weights;
    mutable std::vector<std::string> m_state_names;
};

} // namespace OpenSim

#endif // MOCO_MOCOSUMSQUAREDSTATEGOAL_H
