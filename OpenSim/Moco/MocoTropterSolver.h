#ifndef OPENSIM_MOCOTROPTERSOLVER_H
#define OPENSIM_MOCOTROPTERSOLVER_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoTropterSolver.h                                               *
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

#include "MocoDirectCollocationSolver.h"

#include <SimTKcommon/internal/ResetOnCopy.h>

/** @cond */
namespace tropter {
template <typename T>
class Problem;
template <typename T>
class DirectCollocationSolver;
}
/** @endcond */

namespace OpenSim {

class MocoProblem;

class MocoTropterSolverNotAvailable : public Exception {
public:
    MocoTropterSolverNotAvailable(
            const std::string& file, int line, const std::string& func)
            : Exception(file, line, func) {
        addMessage("MocoTropterSolver is not available.");
    }
};

/** Solve the MocoProblem using the **tropter** direct collocation library.
**tropter** is a free and open-source C++ library that supports computing
the Jacobian and Hessian via either automatic differentiation or finite
differences, and uses IPOPT for solving the nonlinear optimization problem.

This class allows you to configure tropter's settings.

Supported optimization solvers
==============================
The following optimization solvers can be specified for the optim_solver
property:
- ipopt
- snopt

Using this solver in C++ requires that a tropter shared library is
available, but tropter header files are not required. No tropter symbols
are exposed in Moco's interface. */
class OSIMMOCO_API MocoTropterSolver : public MocoDirectCollocationSolver {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MocoTropterSolver, MocoDirectCollocationSolver);

public:
    OpenSim_DECLARE_PROPERTY(optim_jacobian_approximation, std::string,
            "When using IPOPT, 'finite-difference-values' for Jacobian "
            "calculations "
            "by the solver, or 'exact' for Jacobian calculations by "
            "tropter (default).");
    OpenSim_DECLARE_PROPERTY(optim_sparsity_detection, std::string,
            "Trajectory used to detect sparsity pattern of Jacobian/Hessian; "
            "'random' (default) or 'initial-guess'");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(exact_hessian_block_sparsity_mode,
            std::string,
            "'dense' for dense blocks on the Hessian diagonal, or "
            "'sparse' for sparse blocks on the Hessian diagonal, detected from "
            "the "
            "optimal control problem. If using an 'exact' Hessian "
            "approximation, this "
            "property must be set. Note: this option only takes effect when "
            "using "
            "IPOPT.");

    MocoTropterSolver();

    /// Returns true if Moco was compiled with the Tropter library; returns
    /// false otherwise.
    static bool isAvailable();

    /// @name Specifying an initial guess
    /// @{

    /// Create a guess that you can edit and then set using setGuess().
    /// The types of guesses available are:
    /// - **bounds**: variable values are the midpoint between the variables'
    ///   bounds (the value for variables with ony one bound is the specified
    ///   bound). This is the default type.
    /// - **random**: values are randomly generated within the bounds.
    /// - **time-stepping**: see MocoSolver::createGuessTimeStepping().
    /// @note Calling this method does *not* set an initial guess to be used
    /// in the solver; you must call setGuess() or setGuessFile() for that.
    /// @precondition You must have called resetProblem().
    MocoTrajectory createGuess(const std::string& type = "bounds") const;

    // TODO document; any validation?
    /// The number of time points in the trajectory does *not* need to match
    /// `num_mesh_intervals`; the trajectory will be interpolated to the correct
    /// size.
    /// If you have updated the problem since the solver was initialized, you
    /// may need to invoke MocoSolver::resetProblem() for the provided guess to
    /// be recognized as compatible with the problem.
    /// This clears the `guess_file`, if one exists.
    void setGuess(MocoTrajectory guess);
    /// Use this convenience function if you want to choose the type of guess
    /// used, but do not want to modify it first.
    void setGuess(const std::string& type) { setGuess(createGuess(type)); }
    /// This clears any previously-set guess, if any. The file is not loaded
    /// until solving or if you call getGuess().
    /// Set to an empty string to clear the guess file.
    void setGuessFile(const std::string& file);

    /// Clear the stored guess and the `guess_file` if any.
    void clearGuess();

    /// Access the guess, loading it from the guess_file if necessary.
    /// This throws an exception if you have not set a guess (or guess file).
    /// If you have not set a guess (or guess file), this returns an empty
    /// guess, and when solving, we will generate a guess using bounds.
    const MocoTrajectory& getGuess() const;

    /// @}

    /// Print the available options for the underlying optimization solver.
    static void printOptimizationSolverOptions(std::string solver = "ipopt");

protected:
    /// Internal tropter optimal control problem.
    template <typename T>
    class OCProblem;

    template <typename T>
    class TropterProblemBase;
    template <typename T>
    class ExplicitTropterProblem;
    template <typename T>
    class ImplicitTropterProblem;

    std::shared_ptr<const TropterProblemBase<double>>
    createTropterProblem() const;
    std::unique_ptr<tropter::DirectCollocationSolver<double>>
    createTropterSolver(
            std::shared_ptr<const TropterProblemBase<double>>
                    ocp) const;

    MocoSolution solveImpl() const override;

    /// Check that the provided guess is compatible with the problem and this
    /// solver.
    void checkGuess(const MocoTrajectory& guess) const;

private:
    void constructProperties();

    // When a copy of the solver is made, we want to keep any guess specified
    // by the API, but want to discard anything we've cached by loading a file.
    MocoTrajectory m_guessFromAPI;
    mutable SimTK::ResetOnCopy<MocoTrajectory> m_guessFromFile;
    mutable SimTK::ReferencePtr<const MocoTrajectory> m_guessToUse;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOTROPTERSOLVER_H
