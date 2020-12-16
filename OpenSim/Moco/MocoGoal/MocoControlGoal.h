#ifndef OPENSIM_MOCOCONTROLGOAL_H
#define OPENSIM_MOCOCONTROLGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoControlGoal.h                                                 *
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

#include <OpenSim/Moco/MocoWeightSet.h>
#include "MocoGoal.h"

namespace OpenSim {

/** Minimize the sum of the absolute value of the controls raised to a given
exponent, integrated over the phase. The default weight for each control is
1.0; this can be changed by calling setWeight() or editing the
`control_weights` property in XML.
The exponent must be an integer greater than or equal to 2,
and is 2 by default.
If conducting a predictive simulation, you likely want to set
`divide_by_displacement` to true; otherwise, this cost is minimized by not
moving. Dividing by displacement leads to a quantity similar to cost of
transport.

This goal is computed as follows:

\f[
\frac{1}{d} \int_{t_i}^{t_f} \sum_{c \in C} w_c |x_c(t)|^p ~dt
\f]
We use the following notation:
- \f$ d \f$: displacement of the system, if `divide_by_displacement` is
  true; 1 otherwise.
- \f$ C \f$: the set of control signals.
- \f$ w_c \f$: the weight for control \f$ c \f$.
- \f$ x_c(t) \f$: control signal \f$ c \f$.
- \f$ p \f$: the `exponent`.

If `p > 2`, we first take the absolute value of the control; this is to properly
handle odd exponents.
@ingroup mocogoal */
class OSIMMOCO_API MocoControlGoal : public MocoGoal {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoControlGoal, MocoGoal);

public:
    MocoControlGoal();
    MocoControlGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoControlGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }
    /// Set the weight to use for the term in the cost associated with
    /// `controlName` (the name or path of the corresponding actuator). To
    /// remove a control from the cost function, set its weight to 0. If a
    /// weight is already set for the requested state, then the provided
    /// weight replaces the previous weight. Only controls with non-zero weights
    /// that are associated with actuators for which appliesForce is True are
    /// included in the cost function. Weights set here take precedence over
    /// weights specified with a regular expression.
    void setWeightForControl(
            const std::string& controlName, const double& weight);

    /// Set weights for all controls whose entire path matches the provided
    /// regular expression pattern.
    /// Multiple pairs of patterns and weights can be provided by calling this
    /// function multiple times.
    /// If a control matches multiple patterns, the weight associated with the
    /// last pattern is used.
    void setWeightForControlPattern(
            const std::string& pattern, const double& weight);

    /// Set the exponent on the control signals.
    void setExponent(int exponent) { set_exponent(exponent); }
    double getExponent() const { return get_exponent(); }

    /** Set if the goal should be divided by the displacement of the system's
    center of mass over the phase. */
    void setDivideByDisplacement(bool tf) { set_divide_by_displacement(tf); }
    bool getDivideByDisplacement() const {
        return get_divide_by_displacement();
    }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, SimTK::Real& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;
    void printDescriptionImpl() const override;

private:
    void constructProperties();
    OpenSim_DECLARE_PROPERTY(control_weights, MocoWeightSet,
            "The weights for each control; "
            "the weight for unspecified controls is 1.");
    OpenSim_DECLARE_PROPERTY(control_weights_pattern, MocoWeightSet,
            "Set control weights for all controls matching a regular "
            "expression.")
    OpenSim_DECLARE_PROPERTY(
            exponent, int, "The exponent on controls; greater than or equal to "
                           "2 (default: 2).");
    OpenSim_DECLARE_PROPERTY(divide_by_displacement, bool,
            "Divide by the model's displacement over the phase (default: "
            "false)");
    mutable std::vector<double> m_weights;
    mutable std::vector<int> m_controlIndices;
    mutable std::vector<std::string> m_controlNames;
    mutable std::function<double(const double&)> m_power_function;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOCONTROLGOAL_H
