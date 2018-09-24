/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoConstraint.cpp                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include "MucoConstraint.h"

using namespace OpenSim;
using SimTK::ConstraintIndex;

// ============================================================================
// MucoConstraint
// ============================================================================

MucoConstraint::MucoConstraint() {
    constructProperties();
    if (getName().empty()) setName("constraint");
}

void MucoConstraint::constructProperties() {
    constructProperty_lower_bounds();
    constructProperty_upper_bounds();
    constructProperty_suffixes();
}

// ============================================================================
// MucoSimbodyConstraint
// ============================================================================

MucoSimbodyConstraint::MucoSimbodyConstraint() {
    constructProperties();
}

void MucoSimbodyConstraint::constructProperties() {
    constructProperty_constraint_index(0);
    constructProperty_enforce_position_level_only(true);
}

void MucoSimbodyConstraint::initializeImpl() const {
    
    auto& matter = getModel().getMatterSubsystem();
    const SimTK::Constraint& constraint 
        = matter.getConstraint(ConstraintIndex(get_constraint_index()));

    // Throw error if constraint is not enabled in model.
    // TODO do somewhere else?
    const SimTK::State state = getModel().getWorkingState();
    if (constraint.isDisabled(state)) {
        OPENSIM_THROW(Exception, "Constraint is not enabled in model.");
    }

    // Get number of equations of each type: position, velocity, and 
    // acceleration.
    constraint.getNumConstraintEquationsInUse(state, m_num_position_eqs, 
        m_num_velocity_eqs, m_num_acceleration_eqs);

    if (get_enforce_position_level_only()) {
        OPENSIM_THROW_IF(m_num_velocity_eqs != 0, Exception, "Only holonomic "
            "(position-level) constraints are supported in this mode. "
            "There are " + std::to_string(m_num_velocity_eqs) + 
            " velocity-level scalar constraints associated with the model "
            "Constraint at ConstraintIndex " + 
            std::to_string(get_constraint_index()) + ".");
        OPENSIM_THROW_IF(m_num_acceleration_eqs != 0, Exception, "Only "
            "holonomic (position-level) constraints are supported in this "
            "mode. There are " + std::to_string(m_num_acceleration_eqs) + 
            " acceleration-level scalar constraints associated with the model "
            "Constraint at ConstraintIndex " 
            + std::to_string(get_constraint_index()) + ".");

        m_num_equations = m_num_position_eqs;
    } else {
        // The number of scalar constraint equations equals each of the
        // position, velocity, and acceleration-level constraints specified by
        // the Simbody constraint plus the first derivatives of the position and 
        // velocity-level constraint equations and the second derivatives of the
        // position-level constraint equations.
        m_num_equations = 3*m_num_position_eqs + 2*m_num_velocity_eqs + 
            m_num_acceleration_eqs;
    }

    // Assign reference pointer to the model constraint.
    m_constraint_ref = constraint;

    // Set default properties from the model if not already specified by the
    // user.
    if (getName().empty()) {
        const std::string& name = getModel().getConstraintSet().get(
            get_constraint_index()).getName();
        (const_cast <MucoSimbodyConstraint*> (this))->setName(name);
    }
    if (get_suffixes().empty()) {
        std::vector<std::string> suffixes;
        if (get_enforce_position_level_only()) {
            for (int i = 0; i < m_num_position_eqs; ++i) {
                suffixes.push_back("_p" + std::to_string(i));
            }
        } else {
            // Constraint errors order (ref. Simbody::Constraint): 
            // [p1 p2 ... dp1 dp2 ... v1 v2 ... ddp1 ddp2 ... dv1 dv2 ... a1 a2 ...]

            // Position-level constraints.
            for (int i = 0; i < m_num_position_eqs; ++i) {
                suffixes.push_back("_p" + std::to_string(i));
            }
            // Derivative of position-level and velocity-level constraints. 
            for (int i = 0; i < m_num_position_eqs; ++i) {
                suffixes.push_back("_dp" + std::to_string(i));
            }
            for (int i = 0; i < m_num_velocity_eqs; ++i) {
                suffixes.push_back("_v" + std::to_string(i));
            }
            // Second derivative of position-level, derivative of velocity-
            // level, and acceleration-level constraints.
            for (int i = 0; i < m_num_position_eqs; ++i) {
                suffixes.push_back("_ddp" + std::to_string(i));
            }
            for (int i = 0; i < m_num_velocity_eqs; ++i) {
                suffixes.push_back("_dv" + std::to_string(i));
            }
            for (int i = 0; i < m_num_acceleration_eqs; ++i) {
                suffixes.push_back("_dv" + std::to_string(i));
            }
        }
        (const_cast <MucoSimbodyConstraint*> (this))->set_suffixes(suffixes);
    }

    // Set lower and upper bounds to zero.
    SimTK::Vector zeros(m_num_equations, 0.0);
    (const_cast <MucoSimbodyConstraint*> (this))->set_lower_bounds(zeros);
    (const_cast <MucoSimbodyConstraint*> (this))->set_upper_bounds(zeros);
}

void MucoSimbodyConstraint::calcConstraintErrorsImpl(const SimTK::State& state,
    SimTK::Vector& errors) const {

    if (get_enforce_position_level_only()) {
        errors = m_constraint_ref->getPositionErrorsAsVector(state);
    } else { 
        // Constraint errors order (ref. Simbody::Constraint): 
        // [p1 p2 ... dp1 dp2 ... v1 v2 ... ddp1 ddp2 ... dv1 dv2 ... a1 a2 ...]

        // Position-level constraint errors.
        perr = m_constraint_ref->getPositionErrorsAsVector(state);
        std::copy(perr.begin(), perr.end(),
                  errors.begin());
        // Derivative of position-level and velocity-level constraint errors.
        pverr = m_constraint_ref->getVelocityErrorsAsVector(state);
        std::copy(pverr.begin(), pverr.end(),
            errors.begin() + m_num_position_eqs);
        // Second derivative of position-level, derivative of velocity-level,
        // and acceleration-level constraint errors.
        pvaerr = m_constraint_ref->getAccelerationErrorsAsVector(state);
        std::copy(pvaerr.begin(), pvaerr.end(),
            errors.begin() + 2*m_num_position_eqs + m_num_velocity_eqs);
    } 
}