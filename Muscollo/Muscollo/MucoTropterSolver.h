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
class Problem;
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
    OpenSim_DECLARE_PROPERTY(optim_max_iterations, int,
    "Maximum number of iterations in the optimization solver "
    "(-1 for solver's default).");
    OpenSim_DECLARE_PROPERTY(optim_convergence_tolerance, double,
    "Tolerance used to determine if the objective is minimized "
    "(-1 for solver's default)");
    OpenSim_DECLARE_PROPERTY(optim_constraint_tolerance, double,
    "Tolerance used to determine if the constraints are satisfied "
    "(-1 for solver's default)");
    OpenSim_DECLARE_PROPERTY(optim_hessian_approximation, std::string,
    "'limited-memory' (default) for quasi-Newton, or 'exact' for full Newton.");
    OpenSim_DECLARE_PROPERTY(optim_ipopt_print_level, int,
    "IPOPT's verbosity (see IPOPT documentation).");
    // TODO must make more general for multiple phases, mesh refinement.
    // TODO mesh_point_frequency if time is fixed.

    MucoTropterSolver();

    explicit MucoTropterSolver(const MucoProblem& problem);

    /// @name Specifying an initial guess
    /// @{

    /// Create a guess that you can edit and then set using setGuess().
    /// The available types of guesses depend on the solver.
    /// Leave out the type argument to use the solver's default type.
    /// The types of guesses available are (argument to MucoSolver::setGuess()):
    /// - **bounds**: variable values are the midpoint between the variables'
    ///   bounds (the value for variables with ony one bound is the specified
    ///   bound). This is the default type.
    /// - **random**: values are randomly generated within the bounds.
    /// - **time-stepping**: see createGuessTimeStepping().
    /// @note Calling this method does *not* set an initial guess to be used
    /// in the solver; you must call setGuess() or setGuessFile() for that.
    /// @precondition You must have called setProblem().
    // TODO problem must be upToDate()?
    MucoIterate createGuess(const std::string& type = "bounds") const;

    /// (Experimental) Run a forward simulation (using the OpenSim Manager,
    /// which uses a SimTK::Integrator), using the default controls for
    /// actuators and the default states as the initial states, to create a
    /// guess that is dynamically consistent (constraint errors should be
    /// small). The time range for the simulation is the upper bound on the
    /// initial time and the lower bound on the final time. The initial state
    /// values are the default state values unless:
    ///  - initial bounds are an equality constraint: use the bound value
    ///  - default value is not within the initial bounds: use midpoint of
    ///    initial bounds.
    ///
    /// The number of times in the iterate is the number of successful
    /// integration steps.
    ///
    /// @note This function does not yet support problems with parameters.
    ///
    /// @throws Exception If the lower bound on the final time is less than or
    /// equal to the upper bound on the initial time. This situation is okay in
    /// general; it's just that this function doesn't support it.
    ///
    /// @precondition You must have called setProblem().
    // TODO problem must be upToDate()?
    MucoIterate createGuessTimeStepping() const;

    // TODO document; any validation?
    /// The number of time points in the iterate does *not* need to match
    /// `num_mesh_points`; the iterate will be interpolated to the correct size.
    /// This clears the `guess_file`, if any.
    void setGuess(MucoIterate guess);
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
    const MucoIterate& getGuess() const;

    /// @}

    /// Print the available options for the underlying optimization solver.
    static void printOptimizationSolverOptions(std::string solver = "ipopt");

protected:

    /// Internal tropter optimal control problem.
    template <typename T>
    class OCProblem;

    std::shared_ptr<const tropter::Problem<double>>
    getTropterProblem() const;

    void clearProblemImpl() override;
    void setProblemImpl(const MucoProblem& problem) override;
    // TODO ensure that user-provided guess is within bounds.
    MucoSolution solveImpl() const override;

private:

    OpenSim_DECLARE_PROPERTY(guess_file, std::string,
            "A MucoIterate file storing an initial guess.");

    void constructProperties();

    mutable SimTK::ResetOnCopy<std::shared_ptr<OCProblem<double>>>
            m_tropProblem;

    // When a copy of the solver is made, we want to keep any guess specified
    // by the API, but want to discard anything we've cached by loading a file.
    MucoIterate m_guessFromAPI;
    mutable SimTK::ResetOnCopy<MucoIterate> m_guessFromFile;
    mutable SimTK::ReferencePtr<const MucoIterate> m_guessToUse;
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOTROPTERSOLVER_H
