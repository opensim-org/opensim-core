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

#include <OpenSim/Common/GCVSpline.h>

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

    std::vector<std::string> actuPaths;
    for (const auto& actu : model.getComponentList<Actuator>()) {
        OPENSIM_THROW_IF_FRMOBJ(actu.numControls() != 1, Exception,
                "Currently, only ScalarActuators are supported.");
        actuPaths.push_back(actu.getAbsolutePathString());
    }

    // TODO this assumes controls are in the same order as actuators.
    // The loop that processes weights (two down) assumes that controls are in
    // the same order as actuators. However, the control indices are allocated
    // in the order in which addToSystem() is invoked (not necessarily the order
    // used by getComponentList()). So until we can be absolutely sure that the
    // controls are in the same order as actuators, we run the following check:
    // in order, set an actuator's control signal to NaN and ensure the i-th
    // control is NaN.
    {
        SimTK::Vector nan(1, SimTK::NaN);
        const SimTK::State state = model.getWorkingState();
        int i = 0;
        auto modelControls = model.updControls(state);
        for (const auto& actu : model.getComponentList<Actuator>()) {
            SimTK::Vector origControls(1);
            actu.getControls(modelControls, origControls);
            actu.setControls(nan, modelControls);
            OPENSIM_THROW_IF_FRMOBJ(!SimTK::isNaN(modelControls[i]), Exception,
                    "Internal error: actuators are not in the expected order. "
                    "Submit a bug report.");
            actu.setControls(origControls, modelControls);
            ++i;
        }
    }

    // Make sure there are no weights for nonexistent controls.
    m_hasLower = !getProperty_lower_bound().empty();
    m_hasUpper = !getProperty_upper_bound().empty();
    if (getProperty_control_paths().size() && !m_hasLower && !m_hasUpper) {
        std::cout << "Warning: In MocoControlBoundConstraint '" << getName()
                  << "', control paths are specified but no bounds "
                     " are provided."
                  << std::endl;
    }
    if (m_hasLower || m_hasUpper) {
        for (int i = 0; i < getProperty_control_paths().size(); ++i) {
            const auto& thisName = get_control_paths(i);
            auto loc = std::find(actuPaths.begin(), actuPaths.end(), thisName);
            if (loc == actuPaths.end()) {
                OPENSIM_THROW_FRMOBJ(
                        Exception, "Unrecognized control '" + thisName + "'.");
            }
            m_controlIndices.push_back(
                    (int)std::distance(actuPaths.begin(), loc));
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
                    format("The function's minimum X (%f) must "
                           "be less than or equal to the minimum possible "
                           "initial time (%f).",
                            spline->getMinX(), problemInfo.minInitialTime));
            OPENSIM_THROW_IF_FRMOBJ(
                    spline->getMaxX() < problemInfo.maxFinalTime, Exception,
                    format("The function's maximum X (%f) must "
                           "be greater than or equal to the maximum possible "
                           "final time (%f).",
                            spline->getMaxX(), problemInfo.maxFinalTime));
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
            // lower <= control becomes 0 <= control - lower <= inf
            if (m_hasLower) { bounds.emplace_back(0, SimTK::Infinity); }
            // control <= upper becomes -inf <= control - upper <= 0
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
    int iconstr = 0;
    SimTK::Vector time(1);
    for (const auto& controlIndex : m_controlIndices) {
        const auto& control = controls[controlIndex];
        time[0] = state.getTime();
        // These if-statements work correctly for either value of
        // equality_with_lower.
        if (m_hasLower) {
            errors[iconstr++] = control - get_lower_bound().calcValue(time);
        }
        if (m_hasUpper) {
            errors[iconstr++] = control - get_upper_bound().calcValue(time);
        }
    }
}
