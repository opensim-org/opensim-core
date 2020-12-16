#ifndef OPENSIM_MOCOPERIODICITYGOAL_H
#define OPENSIM_MOCOPERIODICITYGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoPeriodicityGoal.h                                             *
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

/** Create pair of variables for use with a MocoPeriodicityGoal. */
class OSIMMOCO_API MocoPeriodicityGoalPair : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoPeriodicityGoalPair, Object);

public:
    OpenSim_DECLARE_PROPERTY(
            initial_variable, std::string, "Initial variable of the pair.");
    OpenSim_DECLARE_PROPERTY(
            final_variable, std::string, "Final variable of the pair.");
    OpenSim_DECLARE_PROPERTY(negate, bool,
            "Should the initial and final variables have opposite signs? "
            "Defaults to false.");

    MocoPeriodicityGoalPair();
    MocoPeriodicityGoalPair(
            std::string initialVariable, std::string finalVariable);
    MocoPeriodicityGoalPair(std::string initialVariableIsFinalVariable);

private:
    void constructProperties();
};

/** This goal enforces equality between initial and final variable values in
the optimal control problem. The initial and final values can belong to
separate continuous variables or the same continuous variable, as long as
they are the same type of variable (e.g., state or control). Value pairs
are specified via a MocoPeriodicityGoalPair, where the initial variable of
the pair indicates the initial state/control variable and the final
variable indicates the final state/control variable. Set the initial and
final variables to the same variable to enforce periodicity on a single
continuous variable. Only state and control continuous variable pairs are
supported, and are specified via the 'state_pairs' and 'control_pairs'
properties.

To handle initial and final variable values that are equal in absolute value
but differ in sign (e.g. a pelvis rotation in walking), use
addNegatedStatePair or addNegatedControlPair.

To impose bilateral symmetry in a walking simulation,
we can simulate over half a gait cycle and impose periodic constraints. For
bilateral variables (e.g., hip flexion speeds and hamstrings controls), the
constraints should enforce that right and left values, and inversely, should
be the same at the beginning and half of the gait cycle, respectively. For
the other variables (e.g., pelvis tilt values), the constraints should
enforce that the values should be the same at the beginning and half of the
gait cycle. The example code below illustrates how to enforce the
aforementioned constraints with different constructors.
@code
periodicGoal = problem.addGoal<MocoPeriodicityGoal>("periodicGoal");
@endcode
Periodic contraint for the pelvis tilt values:
@code
periodicGoal->addStatePair({ "/jointset/ground_pelvis/pelvis_tilt/value"});
@endcode
Periodic contraints for the hip flexion speeds:
@code
periodicGoal->addStatePair({"/jointset/hip_l/hip_flexion_l/speed",
        "/jointset/hip_r/hip_flexion_r/speed"});
periodicGoal->addStatePair({"/jointset/hip_r/hip_flexion_r/speed",
        "/jointset/hip_l/hip_flexion_l/speed"});
@endcode
Periodic contraints for the hamstrings controls:
@code
MocoPeriodicityGoalPair pair_hamstrings1;
pair_hamstrings1.set_initial_variable("/hamstrings_r");
pair_hamstrings1.set_final_variable("/hamstrings_l");
periodicGoal->append_control_pairs(pair_hamstrings1);
MocoPeriodicityGoalPair pair_hamstrings2;
pair_hamstrings2.set_initial_variable("/hamstrings_l");
pair_hamstrings2.set_final_variable("/hamstrings_r");
periodicGoal->append_control_pairs(pair_hamstrings2);
@endcode
This is an endpoint constraint goal by default.
@ingroup mocogoal */
class OSIMMOCO_API MocoPeriodicityGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoPeriodicityGoal, MocoGoal);

public:
    MocoPeriodicityGoal();
    MocoPeriodicityGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }

    void addStatePair(MocoPeriodicityGoalPair pair) {
        append_state_pairs(std::move(pair));
    }
    void addNegatedStatePair(MocoPeriodicityGoalPair pair) {
        pair.set_negate(true);
        append_state_pairs(std::move(pair));
    }
    void addControlPair(MocoPeriodicityGoalPair pair) {
        append_control_pairs(std::move(pair));
    }
    void addNegatedControlPair(MocoPeriodicityGoalPair pair) {
        pair.set_negate(true);
        append_control_pairs(std::move(pair));
    }

protected:
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    Mode getDefaultModeImpl() const override {
        return Mode::EndpointConstraint;
    }
    void initializeOnModelImpl(const Model& model) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& goal) const override;
    void printDescriptionImpl() const override;

private:
    OpenSim_DECLARE_LIST_PROPERTY(state_pairs, MocoPeriodicityGoalPair,
            "Periodic pairs of states.");
    OpenSim_DECLARE_LIST_PROPERTY(control_pairs, MocoPeriodicityGoalPair,
            "Periodic pairs of controls.");
    void constructProperties();
    mutable std::vector<std::tuple<int, int, int>> m_indices_states;
    mutable std::vector<std::tuple<int, int, int>> m_indices_controls;
    mutable std::vector<std::pair<std::string,std::string>> m_state_names;
    mutable std::vector<std::pair<std::string,std::string>> m_control_names;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOPERIODICITYGOAL_H
