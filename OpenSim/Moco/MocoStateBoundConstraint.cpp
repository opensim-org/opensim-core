//
// Created by Allison John on 7/23/24.
//

#include "MocoStateBoundConstraint.h"

#include "MocoProblemInfo.h"
#include "Components/ControlDistributor.h"

#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Simulation/SimulationUtilities.h>

using namespace OpenSim;

void MocoStateBoundConstraint::constructProperties() {
    constructProperty_state_paths();
    constructProperty_lower_bound();
    constructProperty_upper_bound();
    constructProperty_equality_with_lower(false);
}

void MocoStateBoundConstraint::initializeOnModelImpl(
        const Model& model, const MocoProblemInfo& problemInfo) const {

    auto systemStateIndexMap = createSystemYIndexMap(model);

    m_hasLower = !getProperty_lower_bound().empty();
    m_hasUpper = !getProperty_upper_bound().empty();
    if (!getProperty_state_paths().empty() && !m_hasLower && !m_hasUpper) {
        log_warn("In MocoControlBoundConstraint '{}', control paths are "
                 "specified but no bounds are provided.", getName());
    }
    // Make sure there are no nonexistent states.
    // ignore input control
    if (m_hasLower || m_hasUpper) {
        for (int i = 0; i < getProperty_state_paths().size(); ++i) {
            const auto& thisName = get_state_paths(i);
            OPENSIM_THROW_IF_FRMOBJ(!systemStateIndexMap.count(thisName),
                    Exception,
                    "State path '{}' was provided but no such "
                    "state exists in the model.",
                    thisName)
            m_stateIndices.push_back(systemStateIndexMap[thisName]);
        }
    }

    // stop here

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
