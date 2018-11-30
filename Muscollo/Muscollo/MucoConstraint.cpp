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

// ============================================================================
// MucoConstraintInfo
// ============================================================================

MucoConstraintInfo::MucoConstraintInfo() {
    constructProperties();
    if (getName().empty()) setName("path_constraint");
}

std::vector<std::string> MucoConstraintInfo::getConstraintLabels() const {
    std::vector<std::string> labels(getNumEquations());
    for (int i = 0; i < getNumEquations(); ++i) {
        if (getProperty_suffixes().empty()) {
            labels[i] = getName() + "_" + std::to_string(i);
        }
        else {
            labels[i] = getName() + "_" + get_suffixes(i);
        }
    }
    return labels;
}

void MucoConstraintInfo::printDescription(std::ostream& stream) const {
    stream << getName() << ". " << getConcreteClassName() <<
        ". number of scalar equations: " << getNumEquations();

    const std::vector<MucoBounds> bounds = getBounds();
    stream << ". bounds: ";
    for (int i = 0; i < (int)bounds.size(); ++i) {
        bounds[i].printDescription(stream);
    }
    stream << std::endl;
}

void MucoConstraintInfo::constructProperties() {
    constructProperty_bounds();
    constructProperty_suffixes();
}

// ============================================================================
// MucoMultibodyConstraint
// ============================================================================

MucoMultibodyConstraint::MucoMultibodyConstraint(SimTK::ConstraintIndex cid, 
    int mp, int mv, int ma) {

    // Store Simbody SimTK::ConstraintIndex
    m_simbody_constraint_index = cid;

    // Set the default constraint info name based on the constraint index.
    m_constraint_info.setName("multibody_constraint_cid" + std::to_string(cid));

    // Set the number of constraint equations for each kinematic level.
    m_num_position_eqs = mp;
    m_num_velocity_eqs = mv;
    m_num_acceleration_eqs = ma;

    // The constraint errors to enforced by a solver are ordered as follows:
    //      1) position-level errors (mp length)
    //      2) position-level 1st derivative errors (mp length)
    //      3) velocity-level errors (mv length)
    //      4) position-level 2nd derivative errors (mp length)
    //      5) velocity-level 1st derivative errors (mv length)
    //      6) acceleration-level errors (ma length)
    //
    // Therefore, the *total* number of equations to be enforced is equal to
    //                          3*mp + 2*mv + ma
    m_constraint_info.setNumEquations(3*mp + 2*mv + ma);

    // In general, the default list of suffixes for a Simbody constraint 
    // will take the following pattern:
    //      [p0 p1 ... dp0 dp1 ... v0 v1 ... ddp0 ddp1 ... 
    //       dv0 dv1 ... a0 a1 ...]
    std::vector<std::string> suffixes;

    // Position-level constraints.
    for (int i = 0; i < m_num_position_eqs; ++i) {
        suffixes.push_back("p" + std::to_string(i));
        m_kinematic_levels.push_back(KinematicLevel::Position);
    }
    // Derivative of position-level and velocity-level constraints. 
    for (int i = 0; i < m_num_position_eqs; ++i) {
        suffixes.push_back("dp" + std::to_string(i));
        m_kinematic_levels.push_back(KinematicLevel::DtPosition);
    }
    for (int i = 0; i < m_num_velocity_eqs; ++i) {
        suffixes.push_back("v" + std::to_string(i));
        m_kinematic_levels.push_back(KinematicLevel::Velocity);
    }
    // Second derivative of position-level, derivative of velocity-
    // level, and acceleration-level constraints.
    for (int i = 0; i < m_num_position_eqs; ++i) {
        suffixes.push_back("ddp" + std::to_string(i));
        m_kinematic_levels.push_back(KinematicLevel::DtDtPosition);
    }
    for (int i = 0; i < m_num_velocity_eqs; ++i) {
        suffixes.push_back("dv" + std::to_string(i));
        m_kinematic_levels.push_back(KinematicLevel::DtVelocity);
    }
    for (int i = 0; i < m_num_acceleration_eqs; ++i) {
        suffixes.push_back("a" + std::to_string(i));
        m_kinematic_levels.push_back(KinematicLevel::Acceleration);
    }
    m_constraint_info.setSuffixes(suffixes);
}

void MucoMultibodyConstraint::calcMultibodyConstraintErrors(
        const Model& model,
        const SimTK::State& state,
        SimTK::Vector& errors) const {

    OPENSIM_THROW_IF(
        errors.size() != m_constraint_info.getNumEquations(), Exception,
        "The size of the errors vector passed is not consistent with the "
        "number of scalar equations this MucoMultibodyConstraintInfo "
        "represents.");

    // Get the Simbody constraint.
    const auto& matter = model.getMatterSubsystem();
    const SimTK::Constraint& constraint = 
        matter.getConstraint(getSimbodyConstraintIndex());

    // Position-level constraint errors.
    std::copy_n(constraint.getPositionErrorsAsVector(state).begin(), 
        m_num_position_eqs,
        errors.begin());
    // Derivative of position-level and velocity-level constraint errors.
    std::copy_n(constraint.getPositionErrorsAsVector(state).begin(), 
        m_num_position_eqs + m_num_velocity_eqs,
        errors.begin() + m_num_position_eqs);
    // Second derivative of position-level, derivative of velocity-level,
    // and acceleration-level constraint errors.
    std::copy_n(constraint.getAccelerationErrorsAsVector(state).begin(), 
        m_num_position_eqs + m_num_velocity_eqs + m_num_acceleration_eqs,
        errors.begin() + 2*m_num_position_eqs + m_num_velocity_eqs);
}

// ============================================================================
// MucoPathConstraint
// ============================================================================

MucoPathConstraint::MucoPathConstraint() {
    constructProperties();
    if (getName().empty()) setName("path_constraint");
}

void MucoPathConstraint::constructProperties() {
    constructProperty_MucoConstraintInfo(MucoConstraintInfo());
}

void MucoPathConstraint::initializeOnModel(const Model& model,
        const int& pathConstraintIndex) const {

    m_model.reset(&model);
    initializeOnModelImpl(model);

    OPENSIM_THROW_IF_FRMOBJ(get_MucoConstraintInfo().getNumEquations() <= 0,
        Exception, "Invalid number of equations. Either no equation number was "
        "set or a non-positive integer was provided.");

    OPENSIM_THROW_IF_FRMOBJ(pathConstraintIndex < 0, Exception, "Invalid "
        "constraint index provided. The index must be greater than or equal to "
        "zero.");
    m_path_constraint_index = pathConstraintIndex;    
}
