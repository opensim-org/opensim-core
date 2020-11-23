/* -------------------------------------------------------------------------- *
 *                       OpenSim:  MedialContactForce.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Eastman                                                   *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "MedialContactForce.h"
#include <OpenSim/Common/Component.h>

using namespace OpenSim;

MedialContactForce::MedialContactForce() {
    constructProperties();
}

void MedialContactForce::constructProperties() {
    constructProperty_loads_frame("parent");
    constructProperty_condyle_width(0.05);
    constructProperty_adduction_moment_index(0);
    constructProperty_vertical_force_index(0);
}

void MedialContactForce::extendFinalizeFromProperties() {

    // Get the frame from which the loads are computed.
    checkPropertyValueIsInSet(getProperty_loads_frame(), {"parent", "child"});
    if (get_loads_frame() == "parent") {
        m_isParentFrame = true;
    } else if (get_loads_frame() == "child") {
        m_isParentFrame = false;
    }

    // Check that moment and force indices are valid.
    checkPropertyValueIsInRangeOrSet(
        getProperty_adduction_moment_index(), 0, 2, {});
    checkPropertyValueIsInRangeOrSet(
        getProperty_vertical_force_index(), 0, 2, {});

    m_moment_index = get_adduction_moment_index();
    m_force_index = get_vertical_force_index();
}

void MedialContactForce::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    double force(0.0);
    addCacheVariable<double>(
            "medial_contact_force", force, SimTK::Stage::Acceleration);
}

double MedialContactForce::getMedialContactForce(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, "medial_contact_force")) {
        calcMedialContactForce(s, 
            updCacheVariableValue<double>(s, "medial_contact_force"));
        markCacheVariableValid(s, "medial_contact_force");
    }
    return getCacheVariableValue<double>(s, "medial_contact_force");
}

void MedialContactForce::calcMedialContactForce(const SimTK::State& s, 
        double& force) const {
    
    // Compute the reaction loads on the parent or child frame.
    SimTK::SpatialVec reaction;
    if (m_isParentFrame) {
        reaction = getJoint().calcReactionOnParentExpressedInGround(s);
    } else {
        reaction = getJoint().calcReactionOnChildExpressedInGround(s);
    }

    // Adduction moment.
    const auto& KAM = reaction[0][m_moment_index];
    // Vertical contact force.
    const auto& KCF = reaction[1][m_force_index];

    // Medial contact force.
    force = 0.5 * KCF + (1 / get_condyle_width()) * KAM;
}

