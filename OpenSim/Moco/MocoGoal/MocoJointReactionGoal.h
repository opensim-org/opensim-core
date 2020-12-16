#ifndef OPENSIM_MOCOJOINTREACTIONGOAL_H
#define OPENSIM_MOCOJOINTREACTIONGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoJointReactionGoal.h                                           *
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

#include <OpenSim/Moco/MocoWeightSet.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>

namespace OpenSim {

/** Minimize the sum of squares of specified reaction moment and force
measures for a given joint, integrated over the phase. If the magnitude of
the gravity acceleration vector (Model::get_gravity()) is non-zero, then the
goal is normalized by the model's weight; otherwise, the goal is normalized
by the model's mass. We assume the system's mass is constant (that is,
MocoParameters do not affect mass).

In addition to specifying the joint and reaction measures, the user may
also specify the frame the loads are computed from ("parent" or "child"),
and the frame the loads are expressed in (any valid frame in the model).

Minimizing the y-direction reaction force on the child frame of the right
knee joint expressed in the right tibia frame:
@code
auto* cost = problem.addGoal<MocoJointReactionCost>();
cost->setName("tibiofemoral_compressive_force");
cost->setJointPath("/jointset/knee_r");
cost->setLoadsFrame("child");
cost->setExpressedInFramePath("/bodyset/tibia_r");
cost->setReactionMeasures({"force-y"});
@endcode

This cost requires realizing to the Acceleration stage.
@ingroup mocogoal */
class OSIMMOCO_API MocoJointReactionGoal : public MocoGoal {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoJointReactionGoal, MocoGoal);
public:
    MocoJointReactionGoal();
    MocoJointReactionGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoJointReactionGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    /** Set the model path to the joint whose reaction load(s) will be
    minimized. */
    void setJointPath(const std::string& jointPath)
    {   set_joint_path(jointPath); }
    /** Set the frame from which the reaction loads are computed. Options:
    "parent" or "child" (default: "parent"). */
    void setLoadsFrame(const std::string& frame)
    {   set_loads_frame(frame); }
    /** Set the frame in which the minimized reaction load is expressed. By
    default, it is set to the parent or child frame depending on the
    'loads_frame' property value. */
    void setExpressedInFramePath(const std::string& framePath) 
    {   set_expressed_in_frame_path(framePath); }
    /** Set a specific set of reaction measures to be minimized. Options:
    "moment-x", "moment-y", "moment-z", "force-x", "force-y", and "force-z".
    All reaction measures are minimized by default.
    Replaces the reaction measure set if it already exists. */
    void setReactionMeasures(const std::vector<std::string>& measures){
        updProperty_reaction_measures().clear();
        for (const auto& measure : measures) {
            append_reaction_measures(measure);
        }
    }
    /** Set the weight for an individual reaction measure. If a weight is
    already set for the requested measure, then the provided weight
    replaces the previous weight. An exception is thrown during
    initialization if a weight for an unknown measure is provided. */
    void setWeight(const std::string& stateName, const double& weight) {
        if (get_reaction_weights().contains(stateName)) {
            upd_reaction_weights().get(stateName).setWeight(weight);
        } else {
            upd_reaction_weights().cloneAndAppend({stateName, weight});
        }
    }
    /** Provide a MocoWeightSet to weight the reaction measures in the cost.
    Replaces the weight set if it already exists. */
    void setWeightSet(const MocoWeightSet& weightSet) {
        upd_reaction_weights() = weightSet;
    }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, SimTK::Real& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.integral / m_denominator;
    }
    void printDescriptionImpl() const override;

private:
    OpenSim_DECLARE_PROPERTY(joint_path, std::string, 
            "The model path to the joint whose reaction load(s) will be "
            "minimized.");
    OpenSim_DECLARE_PROPERTY(loads_frame, std::string, 
            "The frame from which the reaction loads are computed. Options: "
            "'child' or 'parent' (default: 'parent').");
    OpenSim_DECLARE_PROPERTY(expressed_in_frame_path, std::string, 
            "The frame in which the minimized reaction load is expressed.");
    OpenSim_DECLARE_LIST_PROPERTY(reaction_measures, std::string,
            "A specific set of reaction measures to be minimized. Options: "
            "'moment-x', 'moment-y', 'moment-z', 'force-x', 'force-y', and "
            "'force-z'. All reaction measures are minimized by default.");
    OpenSim_DECLARE_PROPERTY(reaction_weights, MocoWeightSet,
            "Set of weight objects to weight individual reaction measures in "
            "the cost.");
    
    void constructProperties();

    mutable double m_denominator;
    mutable SimTK::ReferencePtr<const Joint> m_joint;
    mutable SimTK::ReferencePtr<const Frame> m_frame;
    mutable std::vector<std::pair<int, int>> m_measureIndices;
    mutable std::vector<double> m_measureWeights;
    mutable bool m_isParentFrame;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOJOINTREACTIONGOAL_H
