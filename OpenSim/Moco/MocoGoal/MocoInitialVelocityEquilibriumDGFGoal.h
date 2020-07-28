#ifndef OPENSIM_MOCOINITIALVELOCITYEQUILIBRIUMDGFGOAL_H
#define OPENSIM_MOCOINITIALVELOCITYEQUILIBRIUMDGFGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoInitialVelocityEquilibriumDGFGoal.h                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

/** For DeGrooteFregly2016Muscle components with implicit tendon compliance
dynamics, the initial tendon and fiber velocities are determined based the
derivative of the linearized muscle-tendon equilibrium equation described
in Millard et al. 2013 (Appendix, equation A6).
Without this goal, the derivative of normalized tendon force, which is a
control variable in implicit tendon compliance dynamics, may undesirably
start at a very large value if not constrained or minimized (which it is
not by default).
This is an endpoint constraint goal by default.
@note This goal only applies to DeGrooteFregly2016Muscles.
@ingroup mocogoal */
class OSIMMOCO_API MocoInitialVelocityEquilibriumDGFGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MocoInitialVelocityEquilibriumDGFGoal, MocoGoal);

public:
    MocoInitialVelocityEquilibriumDGFGoal() = default;
    MocoInitialVelocityEquilibriumDGFGoal(std::string name)
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
            m_dgfMuscleRefs;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOINITIALVELOCITYEQUILIBRIUMDGFGOAL_H
