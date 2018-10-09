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
// MucoConstraintInfo
// ============================================================================

MucoConstraintInfo::MucoConstraintInfo() {
    constructProperties();
}

std::vector<std::string> MucoConstraintInfo::getConstraintLabels() {
    std::vector<std::string> labels(getNumEquations());
    for (int i = 0; i < getNumEquations(); ++i) {
        if (getProperty_suffixes().empty()) {
            labels[i] = getName() + "_" + std::to_string(i);
        }
        else {
            labels[i] = getName() + get_suffixes(i);
        }
    }
    return labels;
}

void MucoConstraintInfo::printDescription(std::ostream& stream) const {
    stream << getName() << ". " << getConcreteClassName() <<
        ". number of scalar equations : " << getNumEquations();

    const std::vector<MucoBounds> bounds = getBounds();
    stream << ". bounds: ";
    for (int i = 0; i < bounds.size(); ++i) {
        bounds[i].printDescription(stream);
    }
    stream << std::endl;
}

void MucoConstraintInfo::constructProperties() {
    constructProperty_bounds();
    constructProperty_suffixes();
}

// ============================================================================
// MucoMultibodyConstraintInfo
// ============================================================================

MucoMultibodyConstraintInfo::MucoMultibodyConstraintInfo(int cid, int mp, 
        int mv, int ma) {

    // Store Simbody SimTK::ConstraintIndex (internally as int)
    m_simbody_constraint_index = cid;

    // Set the default name based on the constraint index.
    setName("multibody_constraint_cid" + std::to_string(cid));

    // Set the number of constraint equations for each kinematic level.
    m_num_position_eqs = mp;
    m_num_velocity_eqs = mv;
    m_num_acceleration_eqs = ma;

    // The constraint errors enforced by MucoTropterSolver are ordered as
    // follows:
    //      1) position-level errors (mp length)
    //      2) position-level 1st derivative errors (mp length)
    //      3) velocity-level errors (mv length)
    //      4) position-level 2nd derivative errors (mp length)
    //      5) velocity-level 1st derivative errors (mv length)
    //      6) acceleration-level errors (ma length)
    //
    // Therefore, the *total* number of equations to be enforced is equal to
    //                          3*mp + 2*mv + ma

    // Zero-bounds by default.
    std::vector<MucoBounds> bounds;
    for (int i = 0; i < (3*mp + 2*mv + ma); ++i) {
        bounds.push_back({0.0, 0.0});
    }
    // This also sets the internal variable tracking the number of
    // constraint equations.
    setBounds(bounds);

    // In general, the default list of suffixes for a Simbody constraint 
    // will take the following pattern:
    //      [_p0 _p1 ... _dp0 _dp1 ... _v0 _v1 ... _ddp0 _ddp1 ... 
    //       _dv0 _dv1 ... a0 a1 ...]
    std::vector<std::string> suffixes;

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
        suffixes.push_back("_a" + std::to_string(i));
    }
    setSuffixes(suffixes);

}

// ============================================================================
// MucoPathConstraint
// ============================================================================

MucoPathConstraint::MucoPathConstraint() {
    constructProperties();
    if (getName().empty()) setName("path_constraint");
}

void MucoPathConstraint::constructProperties() {
    constructProperty_constraint_info(MucoConstraintInfo());
}

void MucoPathConstraint::initialize(const Model& model, 
        const int& pathConstraintIndex) const {

    m_model.reset(&model);
    initializeImpl();

    OPENSIM_THROW_IF_FRMOBJ(pathConstraintIndex < 0, Exception, "Invalid "
        "constraint index provided. Indices must be greater than or equal to "
        "zero.");
    m_path_constraint_index = pathConstraintIndex;

    OPENSIM_THROW_IF_FRMOBJ(get_constraint_info().getBounds().empty(),
        Exception, "Constraint bounds must be provided within the constraint "
        "info.");

    // Ensure that the user's implementation returns the correct number of 
    // constraint errors.
    SimTK::Vector errors;
    calcPathConstraintErrorsImpl(model.getWorkingState(), errors);
    OPENSIM_THROW_IF_FRMOBJ(errors.size() != 
        get_constraint_info().getNumEquations(), Exception,
        "The length of the constraint errors vector passed from "
        "calcConstraintErrorsImpl() must be consistent with the number of "
        "constraint equations.");
}
