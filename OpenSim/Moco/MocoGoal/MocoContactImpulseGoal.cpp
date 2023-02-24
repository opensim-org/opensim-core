/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoContactImpulseGoal.cpp                                   *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2023 Stanford University and the Authors                     *
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

#include "MocoContactImpulseGoal.h"
#include <OpenSim/Simulation/Model/SmoothSphereHalfSpaceForce.h>

using namespace OpenSim;

void MocoContactImpulseGoal::constructProperties() {
    constructProperty_contact_force_paths();
    constructProperty_impulse_axis(-1);
    constructProperty_extremum_type("none");
    constructProperty_smoothing_factor(1.0);
}

void MocoContactImpulseGoal::initializeOnModelImpl(const Model& model) const {

    // Calculate the denominator.
    m_denominator = model.getTotalMass(model.getWorkingState());
    const double gravityAccelMagnitude = model.get_gravity().norm();
    if (gravityAccelMagnitude > SimTK::SignificantReal) {
        m_denominator *= gravityAccelMagnitude;
    }

    // Store references to contact elements.
    for (int ic = 0; ic < getProperty_contact_force_paths().size();  ++ic) {
        const auto& path = get_contact_force_paths(ic);
        const auto& contactForce =
                model.getComponent<SmoothSphereHalfSpaceForce>(path);

        m_contacts.push_back(&contactForce);
    }

    // Which axis should we optimize the contact impulse along?
    if (get_impulse_axis() < 0 || get_impulse_axis() > 2)
        OPENSIM_THROW_FRMOBJ(Exception,
                "Expected 'impulse_axis' to be either 0, 1 or 2, but "
                "got '{}'.", get_impulse_axis());

    checkPropertyValueIsInSet(
            getProperty_extremum_type(), {"none", "minimum", "maximum"});
    if (get_extremum_type() == "minimum") {
        m_beta = -1;
        m_apply_extremum = true;
    } else if (get_extremum_type() == "maximum") {
        m_beta = 1;
        m_apply_extremum = true;
    } else if (get_extremum_type() == "none") {
        m_apply_extremum = false;
    }

    checkPropertyValueIsInRangeOrSet(
            getProperty_smoothing_factor(), 0.2, 1.0, {});
    m_smoothing_factor = get_smoothing_factor();

    setRequirements(1, 1, SimTK::Stage::Velocity);
}

void MocoContactImpulseGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    const auto& state = input.state;
    const auto& time = state.getTime();
    getModel().realizeVelocity(state);
    SimTK::Vector timeVec(1, time);
    integrand = 0;

    // Model force.
    double contact_force = 0;
    for (const auto& contact : m_contacts) {
        Array<double> recordValues = contact->getRecordValues(state);
        contact_force += recordValues[get_impulse_axis()];
    }

    if (m_apply_extremum) {
        contact_force = 
            (1 / m_smoothing_factor) *
            (std::log(1 + exp(m_smoothing_factor * m_beta * contact_force)));
        integrand = m_beta * contact_force;
    } else {
        integrand = contact_force;
    }
}