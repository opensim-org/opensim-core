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

std::vector<std::string> MucoConstraint::getConstraintLabels() {
    OPENSIM_THROW_IF(!m_num_equations, Exception,
        "Labels are not available until after initialization.");

    std::vector<std::string> labels(m_num_equations);
    for (int i = 0; i < m_num_equations; ++i) {
        if (getProperty_suffixes().empty()) {
            labels[i] = getName() + "_" + std::to_string(i);
        } else {
            labels[i] = getName() + get_suffixes(i);
        }
    }   
}

void MucoConstraint::initialize(const Model& model) const {
    m_model.reset(&model);
    initializeImpl();

    OPENSIM_THROW_IF_FRMOBJ(!m_num_equations, Exception, "The number of "
        "constraint equations has not be defined.");
    OPENSIM_THROW_IF_FRMOBJ(getProperty_lower_bounds().empty() || 
        getProperty_upper_bounds().empty(), Exception, "Constraint bounds must "
        "be provided.");
    OPENSIM_THROW_IF_FRMOBJ(getProperty_lower_bounds().size() !=
        getProperty_lower_bounds().size(), Exception, "The number of lower and "
        "upper bounds must be consistent.");
    OPENSIM_THROW_IF_FRMOBJ(getProperty_lower_bounds().size() != 
        m_num_equations, Exception, "Number of bounds must be consistent with "
        "the number of constraint equations.");
    if (getProperty_suffixes().empty()) {
        OPENSIM_THROW_IF_FRMOBJ(getProperty_suffixes().size() !=
            m_num_equations, Exception, "Number of suffixes must be consistent "
            "with the number of constraint equations.");
    }
}

// ============================================================================
// MucoSimbodyConstraint
// ============================================================================

MucoSimbodyConstraint::MucoSimbodyConstraint() {
    constructProperties();
}

void MucoSimbodyConstraint::constructProperties() {
    constructProperty_constraint_index(-1);
    constructProperty_enforce_position_level_only(false);
}

void MucoSimbodyConstraint::initializeImpl() const {

    OPENSIM_THROW_IF_FRMOBJ(get_constraint_index() < 0, Exception, 
        "A ConstraintIndex must be provided.");
    
    auto& matter = getModel().getMatterSubsystem();
    const SimTK::Constraint& constraint 
        = matter.getConstraint(ConstraintIndex(get_constraint_index()));

    // Throw error if constraint is not enabled in model.
    const SimTK::State state = getModel().getWorkingState();
    OPENSIM_THROW_IF_FRMOBJ(constraint.isDisabled(state), Exception, "The "
        "constraint at ConstraintIndex " 
        + std::to_string(get_constraint_index()) + " is not enabled in model.");
    
    // Get number of equations of each type: position, velocity, and 
    // acceleration.
    constraint.getNumConstraintEquationsInUse(state, m_num_position_eqs, 
        m_num_velocity_eqs, m_num_acceleration_eqs);

    if (get_enforce_position_level_only()) {
        OPENSIM_THROW_IF_FRMOBJ(m_num_velocity_eqs != 0, Exception, "Only " 
            "holonomic (position-level) constraints are supported in this "
            "mode. There are " + std::to_string(m_num_velocity_eqs) + 
            " velocity-level scalar constraints associated with the model "
            "Constraint at ConstraintIndex " + 
            std::to_string(get_constraint_index()) + ".");
        OPENSIM_THROW_IF_FRMOBJ(m_num_acceleration_eqs != 0, Exception, "Only "
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

    // By default, set the name of the constraint to the name set in the model.
    if (getName().empty()) {
        const std::string& name = getModel().getConstraintSet().get(
            get_constraint_index()).getName();
        // TODO avoid const_cast
        (const_cast <MucoSimbodyConstraint*> (this))->setName(name);
    }

    // By default, set the suffixes to according to kinematic level being 
    // enforced by the constraint (position, velocity, or acceleration) and
    // specify if it represents a derivative of a scalar equation.
    // TODO avoid const_cast
    if (getProperty_suffixes().empty()) {
        if (get_enforce_position_level_only()) {
            for (int i = 0; i < m_num_position_eqs; ++i) {
                (const_cast <MucoSimbodyConstraint*> (this))->append_suffixes(
                    "_p" + std::to_string(i));
            }
        } else {
            // The constraint errors returned by calcConstraintErrorsImpl()
            // are ordered as follows:
            //      1) position-level errors
            //      2) position-level 1st derivative errors
            //      3) velocity-level errors
            //      4) position-level 2nd derivative errors
            //      5) velocity-level 1st derivative errors
            //      6) acceleration-level errors
            //      
            // Therefore, in general the list of suffixes for a Simbody 
            // constraint will take the following pattern:
            //      [_p0 _p1 ... _dp0 _dp1 ... _v0 _v1 ... _ddp0 _ddp1 ... 
            //       _dv0 _dv1 ... a0 a1 ...]

            // Position-level constraints.
            for (int i = 0; i < m_num_position_eqs; ++i) {
                (const_cast <MucoSimbodyConstraint*> (this))->append_suffixes(
                    "_p" + std::to_string(i));
            }
            // Derivative of position-level and velocity-level constraints. 
            for (int i = 0; i < m_num_position_eqs; ++i) {
                (const_cast <MucoSimbodyConstraint*> (this))->append_suffixes(
                    "_dp" + std::to_string(i));
            }
            for (int i = 0; i < m_num_velocity_eqs; ++i) {
                (const_cast <MucoSimbodyConstraint*> (this))->append_suffixes(
                    "_v" + std::to_string(i));
            }
            // Second derivative of position-level, derivative of velocity-
            // level, and acceleration-level constraints.
            for (int i = 0; i < m_num_position_eqs; ++i) {
                (const_cast <MucoSimbodyConstraint*> (this))->append_suffixes(
                    "_ddp" + std::to_string(i));
            }
            for (int i = 0; i < m_num_velocity_eqs; ++i) {
                (const_cast <MucoSimbodyConstraint*> (this))->append_suffixes(
                    "_dv" + std::to_string(i));
            }
            for (int i = 0; i < m_num_acceleration_eqs; ++i) {
                (const_cast <MucoSimbodyConstraint*> (this))->append_suffixes(
                    "_a" + std::to_string(i));
            }
        }
    } 

    // Set lower and upper bounds to zero.
    SimTK::Vector zeros(m_num_equations, 0.0);
    // TODO avoid const_cast
    (const_cast <MucoSimbodyConstraint*> (this))->set_lower_bounds(zeros);
    (const_cast <MucoSimbodyConstraint*> (this))->set_upper_bounds(zeros);
}

void MucoSimbodyConstraint::calcConstraintErrorsImpl(const SimTK::State& state,
    SimTK::Vector& errors) const {

    if (get_enforce_position_level_only()) {
        errors = m_constraint_ref->getPositionErrorsAsVector(state);
    } else { 
        // The constraint errors returned are ordered as follows:
        //      1) position-level errors
        //      2) position-level 1st derivative errors
        //      3) velocity-level errors
        //      4) position-level 2nd derivative errors
        //      5) velocity-level 1st derivative errors
        //      6) acceleration-level errors

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