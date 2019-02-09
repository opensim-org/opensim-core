/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoCasOCFunction.cpp                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2018 Stanford University and the Authors                     *
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

#include "CasOCFunction.h"

#include "CasOCProblem.h"

using namespace CasOC;

casadi::Sparsity PathConstraint::get_sparsity_in(casadi_int i) {
    if (i == 0) {
        return casadi::Sparsity::dense(1, 1);
    } else if (i == 1) {
        return casadi::Sparsity::dense(m_casProblem->getNumStates(), 1);
    } else if (i == 2) {
        return casadi::Sparsity::dense(m_casProblem->getNumControls(), 1);
    } else if (i == 3) {
        return casadi::Sparsity::dense(m_casProblem->getNumParameters(), 1);
    } else {
        return casadi::Sparsity(0, 0);
    }
}

casadi::Sparsity IntegralCostIntegrand::get_sparsity_in(casadi_int i) {
    if (i == 0) {
        return casadi::Sparsity::dense(1, 1);
    } else if (i == 1) {
        return casadi::Sparsity::dense(m_casProblem->getNumStates(), 1);
    } else if (i == 2) {
        return casadi::Sparsity::dense(m_casProblem->getNumControls(), 1);
    } else if (i == 3) {
        return casadi::Sparsity::dense(m_casProblem->getNumParameters(), 1);
    } else {
        return casadi::Sparsity(0, 0);
    }
}

casadi::Sparsity EndpointCost::get_sparsity_in(casadi_int i) {
    // TODO: Detect this sparsity.
    if (i == 0) {
        return casadi::Sparsity::scalar();
    } else if (i == 1) {
        return casadi::Sparsity::dense(m_casProblem->getNumStates(), 1);
    } else if (i == 2) {
        return casadi::Sparsity::dense(m_casProblem->getNumParameters(), 1);
    } else {
        return casadi::Sparsity(0, 0);
    }
}

casadi::Sparsity MultibodySystem::get_sparsity_in(casadi_int i) {
    if (i == 0) {
        return casadi::Sparsity::dense(1, 1);
    } else if (i == 1) {
        return casadi::Sparsity::dense(m_casProblem->getNumStates(), 1);
    } else if (i == 2) {
        return casadi::Sparsity::dense(m_casProblem->getNumControls(), 1);
    } else if (i == 3) {
        return casadi::Sparsity::dense(m_casProblem->getNumMultipliers(), 1);
    } else if (i == 4) {
        return casadi::Sparsity::dense(m_casProblem->getNumParameters(), 1);
    } else {
        return casadi::Sparsity(0, 0);
    }
}
casadi::Sparsity MultibodySystem::get_sparsity_out(casadi_int i) {
    if (i == 0) {
        return casadi::Sparsity::dense(m_casProblem->getNumSpeeds(), 1);
    } else if (i == 1) {
        return casadi::Sparsity::dense(
                m_casProblem->getNumAuxiliaryStates(), 1);
    } else if (i == 2) {
        int numRows = m_casProblem->getNumKinematicConstraintEquations();
        return casadi::Sparsity::dense(numRows, 1);
    } else
        return casadi::Sparsity(0, 0);
}
