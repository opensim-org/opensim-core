#ifndef MOCO_MOCOCONTROLGOAL_H
#define MOCO_MOCOCONTROLGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoControlGoal.h                                            *
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

#include "../MocoWeightSet.h"
#include "MocoGoal.h"

namespace OpenSim {

/// Minimize the sum of squared controls, integrated over the phase.
/// The default weight for each control is 1.0; this can be changed by
/// calling setWeight() or editing the `control_weights` property in XML.
/// You can choose the exponent TODO
/// If conducting a predictive simulation, you likely want to set
/// `divide_by_displacement` to true; otherwise, this cost is minimized by not
/// moving. Dividing by displacement leads to a quantity similar to cost of
/// transport.
/// @ingroup mocogoal
// TODO allow leaving out some controls.
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
    /// included in the cost function.
    void setWeightForControl(const std::string& controlName, const double& weight);

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const SimTK::State& state, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;

private:
    void constructProperties();
    OpenSim_DECLARE_PROPERTY(control_weights, MocoWeightSet,
            "The weights for each control; "
            "the weight for unspecified controls is 1.");
    OpenSim_DECLARE_PROPERTY(exponent, int, "TODO");
    OpenSim_DECLARE_PROPERTY(divide_by_displacement, bool,
            "Divide by the model's displacement over the phase.");
    mutable std::vector<double> m_weights;
    mutable std::vector<int> m_controlIndices;
    mutable int m_exponent = -1;
};

} // namespace OpenSim

#endif // MOCO_MOCOCONTROLGOAL_H
