#ifndef MOCO_MOCOPERIODICITYGOAL_H
#define MOCO_MOCOPERIODICITYGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoPeriodicityGoal.h                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-19 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Antoine Falisse                                                 *
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

/// Create pair of elements for use when imposing periodic constraints.
class OSIMMOCO_API MocoPeriodicityGoalPair : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoPeriodicityGoalPair, Object);

public:
    OpenSim_DECLARE_PROPERTY(
            first, std::string, "First element of the pair.");
    OpenSim_DECLARE_PROPERTY(
            second, std::string, "Second element of the pair.");

    MocoPeriodicityGoalPair();
    MocoPeriodicityGoalPair(std::string name);

private:
    void constructProperties();

};

/// The initial state of the first element of the pair should be the same as
/// the final state of the second element of the pair. Pairs of states should
/// be distinguished from pairs of controls by appending a `state_pair` or
/// a `control_pair` to the MocoPeriodicityGoal.
/// This is an endpoint constraint goal by default.
/// @ingroup mocogoal
class OSIMMOCO_API MocoPeriodicityGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoPeriodicityGoal, MocoGoal);

public:
    OpenSim_DECLARE_LIST_PROPERTY(state_pair, MocoPeriodicityGoalPair,
            "Periodic pair of states.");
    OpenSim_DECLARE_LIST_PROPERTY(control_pair, MocoPeriodicityGoalPair,
            "Periodic pair of controls.");

    MocoPeriodicityGoal();
    MocoPeriodicityGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }

protected:
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    Mode getDefaultModeImpl() const override {
        return Mode::EndpointConstraint;
    }
    void initializeOnModelImpl(const Model& model) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& goal) const override;

private:
    void constructProperties();
    mutable std::vector<std::pair<int, int>> m_indices_states;
    mutable std::vector<std::pair<int, int>> m_indices_controls;
};

} // namespace OpenSim

#endif // MOCO_MOCOPERIODICITYGOAL_H
