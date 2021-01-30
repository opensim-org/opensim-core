#ifndef OPENSIM_MOCOSUMSQUAREDSTATEGOAL_H
#define OPENSIM_MOCOSUMSQUAREDSTATEGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoSumSquaredStateGoal.h                                         *
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

#include <OpenSim/Moco/MocoWeightSet.h>

namespace OpenSim {

/** Minimize the sum of squared states, integrated over the phase. For example,
this can be used to minimize muscle activations, as is done in MocoInverse.

This goal is computed as follows:

\f[
\int_{t_i}^{t_f} \sum_{s \in S} w_s y_s(t)^2 ~dt
\f]
We use the following notation:
- \f$ t_i \f$: the initial time of this phase.
- \f$ t_f \f$: the final time of this phase.
- \f$ S \f$: the set of state variables selected for this goal.
- \f$ w_s \f$: the weight for state variable \f$ s \f$.
- \f$ y_s(t) \f$: state variable \f$ s \f$.

Select which states to minimize by using
a regex pattern with `setPattern()`. Provide weights for each
state through `setWeightSet()` or `setWeightForState()`.
@ingroup mocogoal */
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

    /** Provide a MocoWeightSet to weight the state variables in the cost.
    Replaces the weight set if it already exists. */
    void setWeightSet(const MocoWeightSet& weightSet) {
        upd_state_weights() = weightSet;
    }

    /** Set the weight for an individual state variable. If a weight is
    already set for the requested state, then the provided weight
    replaces the previous weight. An exception is thrown if a weight
    for an unknown state is provided. */
    void setWeightForState(const std::string& stateName, const double& weight) {
        if (get_state_weights().contains(stateName)) {
            upd_state_weights().get(stateName).setWeight(weight);
        } else {
            upd_state_weights().cloneAndAppend({stateName, weight});
        }
    }

    /** Only state paths matching the regular expression are included. The
    regular expression must match the entire state path for a state path to
    be included (that is, we use std::regex_match, not std::regex_search).
    To include only generalized coordinates, use `.*value$`.
    To include generalized coordinates and speeds, use `.*(value|speed)$`.
    To include only activations, use `.*activation$`. */
    void setPattern(std::string pattern) { set_pattern(pattern); }
    /** Unset the pattern, which causes all states to be matched. */
    void clearPattern() { updProperty_pattern().clear(); }
    std::string getPattern() const { return get_pattern(); }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, SimTK::Real& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.integral;
    }
    void printDescriptionImpl() const override;

private:
    OpenSim_DECLARE_PROPERTY(state_weights, MocoWeightSet,
            "Set of weight objects to weight the individual "
            "state variables in the cost.");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(pattern, std::string,
            "If provided, only states matching this regular expression are "
            "included (default: no pattern). If no pattern is provided, then "
            "all states are used.");

    void constructProperties() {
        constructProperty_state_weights(MocoWeightSet());
        constructProperty_pattern();
    }

    /// Return the corresponding weight to a given stateName in the
    /// state_weights set. If it doesn't exist, return 1.0.
    double getStateWeight(const std::string& stateName) const;

    /// The indices in Y corresponding to the provided reference
    /// coordinates.
    mutable std::vector<int> m_sysYIndices;
    mutable std::vector<double> m_state_weights;
    mutable std::vector<std::string> m_state_names;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOSUMSQUAREDSTATEGOAL_H
