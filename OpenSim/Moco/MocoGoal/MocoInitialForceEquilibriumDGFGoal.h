#ifndef OPENSIM_MOCOINITIALFORCEEQUILIBRIUMDGFGOAL_H
#define OPENSIM_MOCOINITIALFORCEEQUILIBRIUMDGFGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoInitialForceEquilibriumDGFGoal.h                              *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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

class DeGrooteFregly2016Muscle;

/** For all DeGrooteFregly2016Muscle%s with explicit tendon compliance dynamics,
constrain (or minimize) the error computed from the muscle-tendon force
equilibrium equation.
This goal ensures that the initial normalized tendon force state variable
is chosen such that equilibrium is satisfied; otherwise, the initial state
may not be valid.
This is an endpoint constraint goal by default.
@note This goal only applies to DeGrooteFregly2016Muscle%s since it relies on 
    the method calcEquilibriumResidual() provided by that class. Calls to 
    calcEquilibriumResidual() set the normalized tendon force derivative
    argument to zero.
@ingroup mocogoal */
class OSIMMOCO_API MocoInitialForceEquilibriumDGFGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MocoInitialForceEquilibriumDGFGoal, MocoGoal);

public:
    MocoInitialForceEquilibriumDGFGoal() = default;
    MocoInitialForceEquilibriumDGFGoal(std::string name)
        : MocoGoal(std::move(name)) {}

protected:
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    Mode getDefaultModeImpl() const override {
        return Mode::EndpointConstraint;
    }
    void initializeOnModelImpl(const Model&) const override;
    void calcGoalImpl(
        const GoalInput& input, SimTK::Vector& goal) const override;

private:
    mutable std::vector<SimTK::ReferencePtr<const DeGrooteFregly2016Muscle>>
            m_muscleRefs;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOINITIALFORCEEQUILIBRIUMDGFGOAL_H
