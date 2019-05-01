#ifndef MOCO_MOCOJOINTREACTIONCOST_H
#define MOCO_MOCOJOINTREACTIONCOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoJointReactionNormCost.h                                  *
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

#include "MocoCost.h"
#include "../MocoWeightSet.h"

#include <OpenSim/Simulation/SimbodyEngine/Joint.h>

namespace OpenSim {

/// Minimize the the sum of squares of specified reaction moment and force 
/// components for a given joint, integrated over the phase.
///
/// In addition to specifying the joint and reaction components, the user may
/// also specify the frame the loads are computed from ("parent" or "child"),
/// and the frame the loads are expressed in (any valid frame in the model).
///
/// Minimizing the y-direction reaction force on the parent frame of the right
/// knee joint expressed in the right tibia frame:
/// @code 
/// auto* cost = problem.addCost<MocoJointReactionCost>();
/// cost->setName("tibiofemoral_compressive_force");
/// cost->setJointPath("/jointset/knee_r");
/// cost->setLoadsFrame("parent");
/// cost->setExpressedInFramePath("/bodyset/tibia_r");
/// cost->setReactionComponents({"force-y"});
/// @endcode
///
/// This cost requires realizing to the Acceleration stage.
/// @ingroup mococost
class OSIMMOCO_API MocoJointReactionCost : public MocoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoJointReactionCost, MocoCost);
public: 
    MocoJointReactionCost();
    MocoJointReactionCost(std::string name) : MocoCost(std::move(name)) {
        constructProperties();
    }
    MocoJointReactionCost(std::string name, double weight)
            : MocoCost(std::move(name), weight) {
        constructProperties();
    }

    /// Set the model path to the joint whose reaction load(s) will be
    /// minimized.
    void setJointPath(const std::string& jointPath)
    {   set_joint_path(jointPath); }
    /// Set the frame from which the reaction loads are computed. Options:
    /// "parent" or "child" (default: "child").
    void setLoadsFrame(const std::string& frame)
    {   set_loads_frame(frame); }
    /// Set the frame in which the minimized reaction load is expressed.
    void setExpressedInFramePath(const std::string& framePath) 
    {   set_expressed_in_frame_path(framePath); }
    /// Set a specific set of reaction components to be minimized. If not set,
    /// all reaction components are minimized by default. Replaces the reaction 
    /// component set if it already exists.
    void setReactionComponents(const std::vector<std::string>& components){
        updProperty_reaction_components().clear();
        for (const auto& component : components) {
            append_reaction_components(component);
        }
    }
    /// Set the weight for an individual reaction component. If a weight is
    /// already set for the requested component, then the provided weight
    /// replaces the previous weight. An exception is thrown if a weight
    /// for an unknown component is provided.
    void setWeight(const std::string& stateName, const double& weight) {
        if (get_reaction_weights().contains(stateName)) {
            upd_reaction_weights().get(stateName).setWeight(weight);
        } else {
            upd_reaction_weights().cloneAndAppend({stateName, weight});
        }
    }
    /// Provide a MocoWeightSet to weight the reaction components in the cost.
    /// Replaces the weight set if it already exists.
    void setWeightSet(const MocoWeightSet& weightSet) {
        upd_reaction_weights() = weightSet;
    }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegralCostImpl(const SimTK::State& state,
            double& integrand) const override;

private:
    OpenSim_DECLARE_PROPERTY(joint_path, std::string, 
            "The model path to the joint whose reaction load(s) will be "
            "minimized.");
    OpenSim_DECLARE_PROPERTY(loads_frame, std::string, 
            "The frame from which the reaction loads are computed. Options: "
            "'child' or 'parent' (default: 'child').");
    OpenSim_DECLARE_PROPERTY(expressed_in_frame_path, std::string, 
            "The frame in which the minimized reaction load is expressed.");
    OpenSim_DECLARE_LIST_PROPERTY(reaction_components, std::string,
            "A specific set of reaction components to be minimized. If not "
            "set, all reaction components are minimized by default.");
    OpenSim_DECLARE_PROPERTY(reaction_weights, MocoWeightSet,
            "Set of weight objects to weight individual reaction components in "
            "the cost.");
    
    void constructProperties();

    mutable SimTK::ReferencePtr<const Joint> m_joint;
    mutable SimTK::ReferencePtr<const Frame> m_frame;
    mutable std::vector<std::pair<int, int>> m_componentIndices;
    mutable std::vector<double> m_componentWeights;
};

} // namespace OpenSim

#endif // MOCO_MOCOJOINTREACTIONCOST_H
