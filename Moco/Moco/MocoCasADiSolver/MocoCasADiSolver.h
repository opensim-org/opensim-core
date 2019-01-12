#ifndef MOCO_MOCOCASADISOLVER_H
#define MOCO_MOCOCASADISOLVER_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MocoCasADiSolver.h                                       *
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

#include "../osimMocoDLL.h"

#include "../MocoSolver.h"

class CasADiTranscription;

namespace OpenSim {

/// @note This solver currently only supports systems for which \f$ \dot{q} = u
/// \f$ (e.g., no quaternions).
class OSIMMOCO_API MocoCasADiSolver : public MocoSolver {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoCasADiSolver, MocoSolver);
public:
    OpenSim_DECLARE_PROPERTY(num_mesh_points, int,
    "The number of mesh points for discretizing the problem (default: 100).");
    OpenSim_DECLARE_PROPERTY(verbosity, int,
    "0 for silent. 1 for only Muscollo's own output. "
    "2 for output from CasADi and the underlying solver (default: 2).");
    OpenSim_DECLARE_PROPERTY(dynamics_mode, std::string,
    "Dynamics are expressed as 'explicit' (default) or 'implicit' "
    "differential equations.");
    OpenSim_DECLARE_PROPERTY(optim_solver, std::string,
    "The optimization solver for CasADi to use (default: ipopt).");
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


    MocoCasADiSolver();

    /// @name Specifying an initial guess
    /// @{

    /// Create a guess that you can edit and then set using setGuess().
    /// The types of guesses available are:
    /// - **bounds**: variable values are the midpoint between the variables'
    ///   bounds (the value for variables with ony one bound is the specified
    ///   bound). This is the default type.
    /// - **random**: values are randomly generated within the bounds.
    /// - **time-stepping**: see MocoSolver::createGuessTimeStepping().
    ///   NOTE: This option does not yet work well for this solver.
    /// @note Calling this method does *not* set an initial guess to be used
    /// in the solver; you must call setGuess() or setGuessFile() for that.
    /// @precondition You must have called resetProblem().
    MocoIterate createGuess(const std::string& type = "bounds") const;

    /// The number of time points in the iterate does *not* need to match
    /// `num_mesh_points`; the iterate will be interpolated to the correct size.
    /// This clears the `guess_file`, if any.
    void setGuess(MocoIterate guess);
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
    const MocoIterate& getGuess() const;

    /// @}

protected:
    void resetProblemImpl(const MocoProblemRep&) const override {}
    MocoSolution solveImpl() const override;

    std::unique_ptr<CasADiTranscription> createCasADiProblem() const;

private:

    OpenSim_DECLARE_PROPERTY(guess_file, std::string,
            "A MocoIterate file storing an initial guess.");

    void constructProperties();

    // When a copy of the solver is made, we want to keep any guess specified
    // by the API, but want to discard anything we've cached by loading a file.
    MocoIterate m_guessFromAPI;
    mutable SimTK::ResetOnCopy<MocoIterate> m_guessFromFile;
    mutable SimTK::ReferencePtr<const MocoIterate> m_guessToUse;
};

} // namespace OpenSim


#endif // MOCO_MOCOCASADISOLVER_H
