#ifndef OPENSIM_MOCOINITIALACTIVATIONGOAL_H
#define OPENSIM_MOCOINITIALACTIVATIONGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoInitialActivationGoal.h                                       *
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

/** For all muscles with activation dynamics, the initial activation and initial
excitation should be the same.
Without this goal, muscle activation may undesirably start at its maximum
possible value in inverse/tracking problems which penalize only excitations
(such activation is "free").
This is an endpoint constraint goal by default.
Credit for using this goal to address excessive initial activation goes to
Jessica Allen.
@ingroup mocogoal */
class OSIMMOCO_API MocoInitialActivationGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoInitialActivationGoal, MocoGoal);

public:
    MocoInitialActivationGoal() = default;
    MocoInitialActivationGoal(std::string name) : MocoGoal(std::move(name)) {}

protected:
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    Mode getDefaultModeImpl() const override {
        return Mode::EndpointConstraint;
    }
    void initializeOnModelImpl(const Model&) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& goal) const override;

private:
    mutable std::vector<std::pair<int, int>> m_indices;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOINITIALACTIVATIONGOAL_H
