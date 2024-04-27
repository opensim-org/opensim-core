/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoControlBoundConstraint.cpp                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

#include "MocoControlBoundConstraint.h"

#include "MocoProblemInfo.h"
#include "Components/ControlDistributor.h"

#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Simulation/SimulationUtilities.h>

using namespace OpenSim;

MocoControlBoundConstraint::MocoControlBoundConstraint() {
    constructProperties();
}

void MocoControlBoundConstraint::constructProperties() {
    constructProperty_control_paths();
    constructProperty_lower_bound();
    constructProperty_upper_bound();
    constructProperty_equality_with_lower(false);
}

void MocoControlBoundConstraint::initializeOnModelImpl(
        const Model& model, const MocoProblemInfo& problemInfo) const {

    // Check that the model controls are in the correct order.
    checkOrderSystemControls(model);
    auto systemControlIndexMap = createSystemControlIndexMap(model);

    // Get the Input control index map.
    auto inputControlIndexMap = getInputControlIndexMap(model);

    m_hasLower = !getProperty_lower_bound().empty();
    m_hasUpper = !getProperty_upper_bound().empty();
    if (!getProperty_control_paths().empty() && !m_hasLower && !m_hasUpper) {
        log_warn("In MocoControlBoundConstraint '{}', control paths are "
                 "specified but no bounds are provided.", getName());
    }
    // Make sure there are no nonexistent controls.
    if (m_hasLower || m_hasUpper) {
        for (int i = 0; i < getProperty_control_paths().size(); ++i) {
            const auto& thisName = get_control_paths(i);
            bool foundControl = systemControlIndexMap.count(thisName);
            bool foundInputControl = inputControlIndexMap.count(thisName);
            OPENSIM_THROW_IF_FRMOBJ(!foundControl && !foundInputControl,
                    Exception,
                    "Control path '{}' was provided but no such "
                    "control or Input control exists in the model.",
                    thisName)

            if (foundControl) {
                m_controlIndices.push_back(systemControlIndexMap[thisName]);
                m_isInputControl.push_back(false);
            } else {
                m_controlIndices.push_back(inputControlIndexMap.at(thisName));
                m_isInputControl.push_back(true);
            }   
        }
    }

    OPENSIM_THROW_IF_FRMOBJ(get_equality_with_lower() && m_hasUpper, Exception,
            "If equality_with_lower==true, upper bound function must not be "
            "set.");
    OPENSIM_THROW_IF_FRMOBJ(get_equality_with_lower() && !m_hasLower, Exception,
            "If equality_with_lower==true, lower bound function must be set.");

    auto checkTimeRange = [&](const Function& f) {
        if (auto* spline = dynamic_cast<const GCVSpline*>(&f)) {
            OPENSIM_THROW_IF_FRMOBJ(
                    spline->getMinX() > problemInfo.minInitialTime, Exception,
                    "The function's minimum domain value ({}) must "
                    "be less than or equal to the minimum possible "
                    "initial time ({}).",
                    spline->getMinX(), problemInfo.minInitialTime);
            OPENSIM_THROW_IF_FRMOBJ(
                    spline->getMaxX() < problemInfo.maxFinalTime, Exception,
                    "The function's maximum domain value ({}) must "
                    "be greater than or equal to the maximum possible "
                    "final time ({}).",
                    spline->getMaxX(), problemInfo.maxFinalTime);
        }
    };
    if (m_hasLower) checkTimeRange(get_lower_bound());
    if (m_hasUpper) checkTimeRange(get_upper_bound());

    int numEqsPerControl;
    if (get_equality_with_lower()) {
        numEqsPerControl = 1;
    } else {
        numEqsPerControl = (int)m_hasLower + (int)m_hasUpper;
    }

    setNumEquations(numEqsPerControl * (int)m_controlIndices.size());

    // TODO: setConstraintInfo() is not really intended for use here.
    MocoConstraintInfo info;
    std::vector<MocoBounds> bounds;
    for (int i = 0; i < (int)m_controlIndices.size(); ++i) {
        if (get_equality_with_lower()) {
            bounds.emplace_back(0, 0);
        } else {
            // The lower and upper bounds on the path constraint must be
            // constants, so we cannot use the lower and upper bound functions
            // directly. Therefore, we use a pair of constraints where the
            // lower/upper bound functions are part of the path constraint
            // functions and the lower/upper bounds for the path constraints are
            // -inf, 0, and/or inf.
            // If a lower bound function is provided, we enforce
            //      lower_bound_function <= control
            // by creating the constraint
            //      0 <= control - lower_bound_function <= inf
            if (m_hasLower) { bounds.emplace_back(0, SimTK::Infinity); }
            // If an upper bound function is provided, we enforce
            //      control <= upper_bound_function
            // by creating the constraint
            //      -inf <= control - upper_bound_function <= 0
            if (m_hasUpper) { bounds.emplace_back(-SimTK::Infinity, 0); }
        }
    }
    info.setBounds(bounds);
    const_cast<MocoControlBoundConstraint*>(this)->setConstraintInfo(info);
}

void MocoControlBoundConstraint::calcPathConstraintErrorsImpl(
        const SimTK::State& state, SimTK::Vector& errors) const {
    getModel().realizeVelocity(state);
    const auto& controls = getModel().getControls(state);
    const auto& input_controls = 
            getModel().getComponentList<ControlDistributor>().begin()
                    ->getControls(state);
    int iconstr = 0;
    int numConstraints = static_cast<int>(m_controlIndices.size());
    SimTK::Vector time(1);
    // const auto& controlIndex : m_controlIndices
    for (int iconstr = 0; iconstr < numConstraints; ++iconstr) {
        int controlIndex = m_controlIndices[iconstr];
        const auto& control = m_isInputControl[iconstr] ? 
                controls[controlIndex] : input_controls[controlIndex];
        time[0] = state.getTime();
        // These if-statements work correctly for either value of
        // equality_with_lower.
        if (m_hasLower) {
            errors[iconstr] = control - get_lower_bound().calcValue(time);
        }
        if (m_hasUpper) {
            errors[iconstr] = control - get_upper_bound().calcValue(time);
        }
    }
}
