#ifndef MUSCOLLO_MUCOTROPTERSOLVER_H
#define MUSCOLLO_MUCOTROPTERSOLVER_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoTropterSolver.h                                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include "MucoSolver.h"

#include <SimTKcommon/internal/ResetOnCopy.h>

namespace tropter {
template <typename T>
class OptimalControlProblem;
}

namespace OpenSim {

class MucoProblem;

class MucoTropterSolver : public MucoSolver {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoTropterSolver, MucoSolver);
public:
    OpenSim_DECLARE_PROPERTY(num_mesh_points, double,
            "The number of mesh points for discretizing the problem "
            "(default: 100).");
    // TODO must make more general for multiple phases, mesh refinement.
    // TODO mesh_point_frequency if time is fixed.

    MucoTropterSolver();

    explicit MucoTropterSolver(const MucoProblem& problem);

protected:

    /// Internal tropter optimal control problem.
    template <typename T>
    class OCProblem;

    std::shared_ptr<const tropter::OptimalControlProblem<double>>
    getOptimalControlProblem() const;

    void resetProblemImpl() override;
    void resetProblemImpl(const MucoProblem& problem) override;
    MucoSolution solveImpl() const override;

private:

    void constructProperties();

    mutable SimTK::ResetOnCopy<std::shared_ptr<OCProblem<double>>>
            _tropProblem;

};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOTROPTERSOLVER_H
