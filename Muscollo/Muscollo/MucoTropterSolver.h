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

/// Solve the MucoProblem using the **tropter** direct collocation library.
/// **tropter** is a free and open-source C++ library that supports computing
/// the Jacobian and Hessian via either automatic differentiation or finite
/// differences, and uses IPOPT for solving the nonlinear optimization problem.
///
/// This class allows you to configure tropter's settings.
///
/// Using this solver in C++ requires that a tropter shared library is
/// available, but tropter header files are not required. No tropter symbols
/// are exposed in Muscollo's interface.
class OSIMMUSCOLLO_API MucoTropterSolver : public MucoSolver {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoTropterSolver, MucoSolver);
public:
    OpenSim_DECLARE_PROPERTY(num_mesh_points, int,
    "The number of mesh points for discretizing the problem (default: 100).");
    OpenSim_DECLARE_PROPERTY(verbosity, int,
    "0 for silent. 1 for only Muscollo's own output. "
    "2 for output from tropter and the underlying solver (default: 2).");
    OpenSim_DECLARE_PROPERTY(optim_solver, std::string,
    "The optimization solver for tropter to use; ipopt (default), or snopt.");
    OpenSim_DECLARE_PROPERTY(optim_ipopt_print_level, int,
    "IPOPT's verbosity (see IPOPT documentation).");
    // TODO must make more general for multiple phases, mesh refinement.
    // TODO mesh_point_frequency if time is fixed.

    MucoTropterSolver();

    explicit MucoTropterSolver(const MucoProblem& problem);

    /// Print the available options for the underlying optimization solver.
    static void printOptimizationSolverOptions(std::string solver = "ipopt");

protected:

    /// Internal tropter optimal control problem.
    template <typename T>
    class OCProblem;

    std::shared_ptr<const tropter::OptimalControlProblem<double>>
    getTropterProblem() const;

    void resetProblemImpl() override;
    void resetProblemImpl(const MucoProblem& problem) override;
    MucoSolution solveImpl() const override;

private:

    void constructProperties();

    mutable SimTK::ResetOnCopy<std::shared_ptr<OCProblem<double>>>
            m_tropProblem;

};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOTROPTERSOLVER_H
