/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoStateTrackingGoal.cpp                                    *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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

#include "MocoStateTrackingGoal.h"

#include "../MocoUtilities.h"

#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

void MocoStateTrackingGoal::initializeOnModelImpl(const Model& model) const {

    // TODO: set relativeToDirectory properly.
    TimeSeriesTable tableToUse = get_reference().process("", &model);

    auto allSplines = GCVSplineSet(tableToUse);

    // Check that there are no redundant columns in the reference data.
    checkRedundantLabels(tableToUse.getColumnLabels());

    // Throw exception if a weight is specified for a nonexistent state.
    auto allSysYIndices = createSystemYIndexMap(model);

    std::regex regex;
    if (getProperty_pattern().size()) {
        regex = std::regex(get_pattern());
    }

    for (int i = 0; i < get_state_weights().getSize(); ++i) {
        const auto& weightName = get_state_weights().get(i).getName();
        if (allSysYIndices.count(weightName) == 0) {
            OPENSIM_THROW_FRMOBJ(Exception,
                    "Weight provided with name '{}' but this is "
                    "not a recognized state.",
                    weightName);
        }
        if (getProperty_pattern().size() &&
                !std::regex_match(weightName, regex)) {
            OPENSIM_THROW_FRMOBJ(Exception,
                    "Weight provided with name '{}' but this name "
                    "does not match the pattern '{}'.",
                    weightName, get_pattern());
        }
    }

    // Populate member variables needed to compute cost. Unless the property
    // allow_unused_references is set to true, an exception is thrown for
    // names in the references that don't correspond to a state variable.
    for (int iref = 0; iref < allSplines.getSize(); ++iref) {
        const auto& refName = allSplines[iref].getName();
        if (allSysYIndices.count(refName) == 0) {
            if (get_allow_unused_references()) {
                continue;
            }
            OPENSIM_THROW_FRMOBJ(
                    Exception, "State reference '{}' unrecognized.", refName);
        }
        if (getProperty_pattern().size() &&
                !std::regex_match(refName, regex)) {
            if (get_allow_unused_references()) {
                continue;
            }
            OPENSIM_THROW_FRMOBJ(Exception,
                    "State reference '{}' does not match the pattern '{}'.",
                    refName, get_pattern());
        }

        m_sysYIndices.push_back(allSysYIndices[refName]);
        double refWeight = 1.0;
        if (get_state_weights().contains(refName)) {
            refWeight = get_state_weights().get(refName).getWeight();
        }
        if (get_scale_weights_with_range()) {
            auto refValue = tableToUse.getDependentColumn(refName);
            double refRange =
                    std::abs(SimTK::max(refValue) - SimTK::min(refValue));
            refWeight *= 1.0 / refRange;
        }
        m_state_weights.push_back(refWeight);
        m_refsplines.cloneAndAppend(allSplines[iref]);
        m_state_names.push_back(refName);
    }

    setRequirements(1, 1, SimTK::Stage::Time);
}

void MocoStateTrackingGoal::calcIntegrandImpl(
        const IntegrandInput& input, SimTK::Real& integrand) const {
    const auto& time = input.time;

    SimTK::Vector timeVec(1, time);

    // TODO cache the reference coordinate values at the mesh points, rather
    // than evaluating the spline.
    integrand = 0;
    for (int iref = 0; iref < m_refsplines.getSize(); ++iref) {
        const auto& modelValue = input.state.getY()[m_sysYIndices[iref]];
        const auto& refValue = m_refsplines[iref].calcValue(timeVec);
        integrand += m_state_weights[iref] * pow(modelValue - refValue, 2);
    }
}

void MocoStateTrackingGoal::printDescriptionImpl() const {
    for (int i = 0; i < (int) m_state_names.size(); i++) {
        log_cout("        state: {}, weight: {}", m_state_names[i],
                m_state_weights[i]);
    }
}

