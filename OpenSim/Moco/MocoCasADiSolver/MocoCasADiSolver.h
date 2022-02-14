#ifndef OPENSIM_MOCOCASADISOLVER_H
#define OPENSIM_MOCOCASADISOLVER_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoCasADiSolver.h                                                *
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

#include <OpenSim/Moco/MocoDirectCollocationSolver.h>

namespace CasOC {
class Solver;
} // namespace CasOC

namespace OpenSim {

class MocoCasOCProblem;

class MocoCasADiSolverNotAvailable : public Exception {
public:
    MocoCasADiSolverNotAvailable(
            const std::string& file, int line, const std::string& func)
            : Exception(file, line, func) {
        addMessage("MocoCasADiSolver is not available.");
    }
};

/** This solver uses the CasADi library (https://casadi.org) to convert the
MocoProblem into a generic nonlinear programming problem. CasADi efficiently
calculcates the derivatives required to solve MocoProblem%s, and may
solve your MocoProblem more quickly that MocoTropterSolver. In general,
we hope that the feature sets of MocoCasADiSolver and MocoTropterSolver
are the same.
Note, however, that parameter optimization problems are implemented much
less efficiently in this solver; for parameter optimization, first try
MocoTropterSolver.

Sparsity
========
Direct collocation is fast because the derivative matrices (Jacobian and
Hessian) in the optimization problem are extremely sparse. By default,
CasADi determines the sparsity pattern of these matrices to be block
patterns: the individual functions that invoke OpenSim are treated as dense,
but this dense pattern is repeated in a sparse way. This is conservative
because we ensure that no "nonzeros" are accidentally treated as "zeros."
However, the problem may solve faster if we discover more "zeros."

See the optim_sparsity_detection setting for more information. In the case
of "random", we use 3 random trajectories and combine the resulting sparsity
patterns. The seed used for these 3 random trajectories is always exactly
the same, ensuring that the sparsity pattern is deterministic.

To explore the sparsity pattern for your problem, set optim_write_sparsity
and run the resulting files with the plot_casadi_sparsity.py Python script.

Finite difference scheme
========================
The "central" finite difference is more accurate but can be 2 times
slower than "forward" (tested on exampleSlidingMass). Sometimes, problems
may struggle to converge with "forward".

Parallelization
===============
By default, CasADi evaluate the integral cost integrand and the
differential-algebraic equations in parallel.
This should work fine for almost all models, but if you have custom model
components, ensure they are threadsafe. Make sure that threads do not
access shared resources like files or global variables at the same time.

You can turn off or change the number of parallel jobs used for individual
problems via either the OPENSIM_MOCO_PARALLEL environment variable (see
getMocoParallelEnvironmentVariable()) or the `parallel` property of this
class. For example, if you plan to solve two problems at the same time on
a machine with 4 processor cores, you could set OPENSIM_MOCO_PARALLEL to 2 to
use all 4 cores.

Note that there is overhead in the parallelization; if you plan to solve
many problems, it is better to turn off parallelization here and parallelize
the solving of your multiple problems using your system (e.g., invoke Moco in
multiple Terminals or Command Prompts).

Note that the `parallel` property overrides the environment variable,
allowing more granular control over parallelization. However, the
parallelization setting does not logically belong as a property, as it does
not affect the solution. We encourage you to use the environment variable
instead, as this allows different users to solve the same problem with the
parallelization they prefer.

Parameter variables
===================
By default, MocoCasADiSolver is much slower than MocoTroperSolver at
handling problems with MocoParameters. Many parameters require invoking
Model::initSystem() to take effect, and this function is expensive (for
CasADi, we must invoke this function for every time point, while in Tropter,
we can invoke the function only once for every NLP iterate). However, if you
know that all parameters in your problem do not require Model::initSystem(),
you can substantially speed up your optimization by setting the
parameters_require_initsystem property to false. Be careful, though: you
will end up with incorrect results if your parameter does indeed require
Model::initSystem(). To protect against this, ensure that you obtain the
same results whether this setting is true or false.

@note The software license of CasADi (LGPL) is more restrictive than that of
the rest of Moco (Apache 2.0).
@note This solver currently only supports systems for which \f$ \dot{q} = u
\f$ (e.g., no quaternions). */
class OSIMMOCO_API MocoCasADiSolver : public MocoDirectCollocationSolver {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MocoCasADiSolver, MocoDirectCollocationSolver);

public:
    OpenSim_DECLARE_PROPERTY(scale_variables_using_bounds, bool,
            "Scale optimization variables based on the difference between "
            "variable lower and upper bounds."
            "Default: False.");
    OpenSim_DECLARE_PROPERTY(parameters_require_initsystem, bool,
            "Do some MocoParameters in the problem require invoking "
            "initSystem() to take effect properly? "
            "This substantially slows down problems with parameter variables "
            "(default: true).");
    OpenSim_DECLARE_PROPERTY(optim_sparsity_detection, std::string,
            "Detect the sparsity pattern of derivatives; 'none' "
            "(for safe block sparsity; default), 'random', or "
            "'initial-guess'.");
    OpenSim_DECLARE_PROPERTY(optim_write_sparsity, std::string,
            "Write files for the sparsity pattern of the gradient, Jacobian, "
            "and Hessian to the working directory using this as a prefix; "
            "empty (default) to not write such files.");
    OpenSim_DECLARE_PROPERTY(optim_finite_difference_scheme, std::string,
            "The finite difference scheme CasADi will use to calculate problem "
            "derivatives (default: 'central').");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(parallel, int,
            "Evaluate integral costs and the differential-algebraic "
            "equations in parallel across grid points? "
            "0: not parallel; 1: use all cores (default); greater than 1: use"
            "this number of parallel jobs. This overrides the OPENSIM_MOCO_PARALLEL "
            "environment variable.");
    OpenSim_DECLARE_PROPERTY(output_interval, int,
            "Write intermediate trajectories to file. 0, the default, "
            "indicates no intermediate trajectories are saved, 1 indicates "
            "each iteration is saved, 5 indicates every fifth iteration is "
            "saved, etc.");

    OpenSim_DECLARE_PROPERTY(minimize_implicit_multibody_accelerations, bool,
            "Minimize the integral of the squared acceleration continuous "
            "variables when using the implicit multibody mode. "
            "Default: false.");
    OpenSim_DECLARE_PROPERTY(implicit_multibody_accelerations_weight, double,
            "The weight on the cost term added if "
            "'minimize_implicit_multibody_accelerations' is enabled."
            "Default: 1.0.");
    OpenSim_DECLARE_PROPERTY(minimize_implicit_auxiliary_derivatives, bool,
            "Minimize the integral of the squared derivative continuous "
            "variables for components with implicit auxiliary dynamics. "
            "Default: false.");
    OpenSim_DECLARE_PROPERTY(implicit_auxiliary_derivatives_weight, double,
            "The weight on the cost term added if "
            "'minimize_implicit_auxiliary_derivatives' is enabled."
            "Default: 1.0.");

    OpenSim_DECLARE_PROPERTY(enforce_path_constraint_midpoints, bool,
            "If the transcription scheme is set to 'hermite-simpson', then "
            "enable this property to enforce MocoPathConstraints at mesh "
            "interval midpoints. Default: false.");

    MocoCasADiSolver();

    /// Returns true if Moco was compiled with the CasADi library; returns false
    /// otherwise.
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
    ///   NOTE: This option does not yet work well for this solver.
    /// @note Calling this method does *not* set an initial guess to be used
    /// in the solver; you must call setGuess() or setGuessFile() for that.
    /// @precondition You must have called resetProblem().
    MocoTrajectory createGuess(const std::string& type = "bounds") const;

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

protected:
    MocoSolution solveImpl() const override;

    std::unique_ptr<MocoCasOCProblem> createCasOCProblem() const;
    std::unique_ptr<CasOC::Solver> createCasOCSolver(
            const MocoCasOCProblem&) const;

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

#endif // OPENSIM_MOCOCASADISOLVER_H
