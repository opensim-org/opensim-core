#ifndef OPENSIM_CASOCTRAPEZOIDAL_H
#define OPENSIM_CASOCTRAPEZOIDAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: CasOCTrapezoidal.h                                                *
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

#include "CasOCTranscription.h"

namespace CasOC {

/// Enforce the differential equations in the problem using a trapezoidal
/// (second-order) approximation. The integral in the objective function is
/// approximated by trapezoidal quadrature.
class Trapezoidal : public Transcription {
public:
    Trapezoidal(const Solver& solver, const Problem& problem)
            : Transcription(solver, problem) {
        createVariablesAndSetBounds(m_solver.getMesh(),
                m_problem.getNumStates(), m_problem.getNumStates(), 2);
    }

private:
    casadi::DM createQuadratureCoefficientsImpl() const override;
    casadi::DM createMeshIndicesImpl() const override;
    casadi::DM createControlIndicesImpl() const override;
    void calcDefectsImpl(const casadi::MXVector& x, 
            const casadi::MXVector& xdot, casadi::MX& defects) const override;
    FlattenedVariableInfo getFlattenedVariableInfo() const override;
};

} // namespace CasOC

#endif // OPENSIM_CASOCTRAPEZOIDAL_H
