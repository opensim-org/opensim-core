/* -------------------------------------------------------------------------- *
* OpenSim Moco: MocoJointReactionCost.h                                  *
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

#include "MocoJointReactionCost.h"
#include "../MocoUtilities.h"
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

MocoJointReactionCost::MocoJointReactionCost() {
    constructProperties();
}

void MocoJointReactionCost::constructProperties() {
    constructProperty_joint_path("");
    constructProperty_loads_frame("child");
    constructProperty_expressed_in_frame_path("");
    constructProperty_reaction_components();
    constructProperty_reaction_weights(MocoWeightSet());
}

void MocoJointReactionCost::initializeOnModelImpl(const Model& model) const {

    // Cache the joint.
    OPENSIM_THROW_IF_FRMOBJ(get_joint_path().empty(), Exception,
        "Empty model joint path detected. Please provide a valid joint path.");
    OPENSIM_THROW_IF_FRMOBJ(!model.hasComponent<Joint>(get_joint_path()),
        Exception,
        format("Joint at path %s not found in the model. "
               "Please provide a valid joint path.", get_joint_path()));
    m_joint = &getModel().getComponent<Joint>(get_joint_path());
  
    checkPropertyInSet(*this, getProperty_loads_frame(), {"parent", "child"});
    std::string expressedInFrame;
    if (get_expressed_in_frame_path().empty()) {
        if (get_loads_frame() == "child") {
            expressedInFrame = m_joint->getChildFrame().getAbsolutePathString();
        } else if(get_loads_frame() == "parent") {
            expressedInFrame = 
                m_joint->getParentFrame().getAbsolutePathString();
        }
    } else {
        expressedInFrame = get_expressed_in_frame_path();
    }

    // Cache the expressed-in frame.
    OPENSIM_THROW_IF_FRMOBJ(!model.hasComponent<Frame>(expressedInFrame),
        Exception,
        format("Frame at path %s not found in the model. "
               "Please provide a valid frame path.",
                expressedInFrame));
    m_frame = &getModel().getComponent<Frame>(expressedInFrame);

    // If user provided no reaction component names, then set all components to
    // to be minimized. Otherwise, loop through user-provided component names
    // and check that they are all accepted components.
    std::vector<std::string> reactionComponents;
    std::vector<std::string> allowedComponents = {"moment-x", "moment-y",
        "moment-z", "force-x", "force-y", "force-z"};
    if (getProperty_reaction_components().empty()) {
        reactionComponents = allowedComponents;    
    } else {
        for (int i = 0; i < getProperty_reaction_components().size(); ++i) {
            if (std::find(allowedComponents.begin(), allowedComponents.end(),
                get_reaction_components(i)) == allowedComponents.end()) {
                OPENSIM_THROW_FRMOBJ(Exception,
                    format("Reaction component '%s' specified not recognized.",
                            get_reaction_components(i)));
            }
            reactionComponents.push_back(get_reaction_components(i));
        }
    }

    // Loop through all reaction components to minimize and get the
    // corresponding SpatialVec indices and weights.
    for (const auto& component :  reactionComponents) {
        if (component == "moment-x") {
            m_componentIndices.push_back({0, 0});
        } else if (component == "moment-y") {
            m_componentIndices.push_back({0, 1});
        } else if (component == "moment-z") {
            m_componentIndices.push_back({0, 2});
        } else if (component == "force-x") {
            m_componentIndices.push_back({1, 0});
        } else if (component == "force-y") {
            m_componentIndices.push_back({1, 1});
        } else if (component == "force-z") {
            m_componentIndices.push_back({1, 2});
        }

        double compWeight = 1.0;
        if (get_reaction_weights().contains(component)) {
            compWeight = get_reaction_weights().get(component).getWeight();
        }
        m_componentWeights.push_back(compWeight);
    }
}

void MocoJointReactionCost::calcIntegralCostImpl(const SimTK::State& state,
        double& integrand) const {

    getModel().realizeAcceleration(state);
    const auto& ground = getModel().getGround();

    // Compute the reaction loads on the parent or child frame.
    SimTK::SpatialVec reactionInGround;
    if (get_loads_frame() == "child") {
        reactionInGround =
            m_joint->calcReactionOnChildExpressedInGround(state);
    } else if (get_loads_frame() == "parent") {
        reactionInGround =
            m_joint->calcReactionOnParentExpressedInGround(state);
    }
        
    // Re-express the reactions into the proper frame and repackage into a new
    // SpatialVec.
    const SimTK::Vec3 moment = ground.expressVectorInAnotherFrame(state,
        reactionInGround[0], *m_frame);
    const SimTK::Vec3 force = ground.expressVectorInAnotherFrame(state,
        reactionInGround[1], *m_frame);
    SimTK::SpatialVec reaction(moment, force);

    // Compute cost.
    integrand = 0;
    for (int i = 0; i < m_componentIndices.size(); ++i) {
        const auto index = m_componentIndices[i];
        const double weight = m_componentWeights[i];
        integrand += weight * pow(reaction[index.first][index.second], 2);
    }
}