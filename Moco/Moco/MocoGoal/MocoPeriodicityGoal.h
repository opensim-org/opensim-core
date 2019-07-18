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

/// Create pair of elements for use with a MocoPeriodicityGoal.
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

/// This goal enforces equality between initial and final state values in the
/// optimal control problem. The initial and final states can belong to
/// separate continuous variables or the same continuous variable, as long as
/// they are the same type of variable. Value pairs are specified via a
/// MocoPeriodicityGoalPair, where the first element of the pair indicates the
/// initial state variable and the second element indicates the final state
/// variable. Set the first and second elements to the same variable to enforce
/// periodicity on a single continuous variable. Only state and control
/// continuous variable pairs are supported, and are specified via the
/// 'state_pairs' and 'control_pairs' properties.
///
/// Imposing periodicity of pelvis tilt values, hip speeds, and hamstrings
/// controls after half a gait cycle assuming symmetry of the gait cycle (i.e.,
/// right and left leg values, and inversely, should be the same at the
/// beginning and half of the gait cycle, respectively):
/// @code
/// auto* periodicGoal = problem.addGoal<MocoPeriodicityGoal>("periodicGoal");
/// MocoPeriodicityGoalPair pair_pelvis_tilt_values;
/// pair_pelvis_tilt_values.set_first("/jointset/Pelvis/pelvis_tilt/value");
/// pair_pelvis_tilt_values.set_second("/jointset/Pelvis/pelvis_tilt/value");
/// MocoPeriodicityGoalPair pair_hip1_speeds;
/// pair_hip1_speeds.set_first("/jointset/hip_l/hip_flexion_l/speed");
/// pair_hip1_speeds.set_second("/jointset/hip_r/hip_flexion_r/speed");
/// MocoPeriodicityGoalPair pair_hip2_speeds;
/// pair_hip2_speeds.set_first("/jointset/hip_r/hip_flexion_r/speed");
/// pair_hip2_speeds.set_second("/jointset/hip_l/hip_flexion_l/speed");
/// MocoPeriodicityGoalPair pair_hamstrings1;
/// pair_hamstrings1.set_first("/hamstrings_r");
/// pair_hamstrings1.set_second("/hamstrings_l");
/// MocoPeriodicityGoalPair pair_hamstrings2;
/// pair_hamstrings2.set_first("/hamstrings_l");
/// pair_hamstrings2.set_second("/hamstrings_r");
/// periodicGoal->append_state_pairs(pair_pelvis_tilt_values);
/// periodicGoal->append_state_pairs(pair_hip1_speeds);
/// periodicGoal->append_state_pairs(pair_hip2_speeds);
/// periodicGoal->append_control_pairs(pair_hamstrings1);
/// periodicGoal->append_control_pairs(pair_hamstrings2);
/// @endcode
///
/// This is an endpoint constraint goal by default.
/// @ingroup mocogoal
class OSIMMOCO_API MocoPeriodicityGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoPeriodicityGoal, MocoGoal);

public:
    OpenSim_DECLARE_LIST_PROPERTY(state_pairs, MocoPeriodicityGoalPair,
            "Periodic pairs of states.");
    OpenSim_DECLARE_LIST_PROPERTY(control_pairs, MocoPeriodicityGoalPair,
            "Periodic pairs of controls.");

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
