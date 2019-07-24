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

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const SimTK::State& state, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.integral;
    }

private:
    void constructProperties();
};

} // namespace OpenSim

#endif // MOCO_MOCOSUMSQUAREDSTATEGOAL_H
