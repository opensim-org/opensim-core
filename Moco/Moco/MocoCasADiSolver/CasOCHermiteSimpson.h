#ifndef MOCO_CASOCHERMITESIMPSON_H
#define MOCO_CASOCHERMITESIMPSON_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: CasOCHermiteSimpson.h                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

#include "CasOCTranscription.h"

namespace CasOC {

/// Enforce the differential equations in the problem using a Hermite-
/// Simpson (third-order) approximation. The integral in the objective
/// function is approximated by Simpson quadrature.
class HermiteSimpson : public Transcription {
public:
    HermiteSimpson(const Solver& solver, const Problem& problem)
            : Transcription(solver, problem, 2 * solver.getNumMeshPoints() - 1,
                      solver.getNumMeshPoints()) {
        createVariablesAndSetBounds();
    }

private:
    casadi::DM createQuadratureCoefficientsImpl() const override;
    casadi::DM createKinematicConstraintIndicesImpl() const override;
    void applyConstraintsImpl(const VariablesMX& vars, const casadi::MX& xdot,
            const casadi::MX& residual, const casadi::MX& kcerr,
            const casadi::MXVector& path) override;
};

} // namespace CasOC

#endif // MOCO_CASOCHERMITESIMPSON_H
