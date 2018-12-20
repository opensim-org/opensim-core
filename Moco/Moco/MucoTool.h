#ifndef MUSCOLLO_MUCOTOOL_H
#define MUSCOLLO_MUCOTOOL_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoTool.h                                               *
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

#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/Model/Model.h>

#include "MocoSolver.h"

namespace OpenSim {

class MocoProblem;
class MocoTropterSolver;

/// The top-level class for solving a custom optimal control problem.
///
/// This class consists of a MocoProblem, which describes the optimal control
/// problem, and a MocoSolver, which describes the numerical method for
/// solving the problem.
///
/// Workflow
/// --------
/// When building a MocoTool programmatically (e.g., in C++), the workflow is as
/// follows:
///
/// 1. Build the MocoProblem (set the model, constraints, etc.).
/// 2. Call MocoTool::initSolver(), which returns a reference to the MocoSolver.
///    After this, you cannot edit the MocoProblem.
/// 3. Edit the settings of the MocoSolver (returned by initSolver()).
/// 4. Call MocoTool::solve(). This returns the MocoSolution.
/// 5. (Optional) Postprocess the solution, perhaps using MocoTool::visualize().
///
/// After calling solve(), you can edit the MocoProblem and/or the MocoSolver.
/// You can then call solve() again, if you wish.
///
/// Saving the tool setup to a file
/// -------------------------------
/// You can save the MocoTool to a file by calling MocoTool::print(), and you
/// can load the setup using MocoTool(const std::string& omucoFile).
/// MocoTool setup files have a `.omuco` extension.
///
/// Solver
/// ------
/// The default and only built-in solver uses the **tropter** direct
/// collocation library. We would like to support users plugging in their own
/// solvers, but there is no timeline for this. If you require additional
/// features or enhancements to the solver, please consider contributing to
/// **tropter**.

// TODO rename to MocoFramework.

class OSIMMUSCOLLO_API MocoTool : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoTool, Object);
public:
    OpenSim_DECLARE_PROPERTY(write_solution, std::string,
    "Provide the folder path (relative to working directory) to which the "
    "solution files should be written. Set to 'false' to not write the "
    "solution to disk.");

    MocoTool();

    /// Load a MocoTool setup file.
    MocoTool(const std::string& omucoFile);

    const MocoProblem& getProblem() const;

    /// If using this method in C++, make sure to include the "&" in the
    /// return type; otherwise, you'll make a copy of the problem, and the copy
    /// will have no effect on this MocoTool.
    MocoProblem& updProblem();

    /// Call this method once you have finished setting up your MocoProblem.
    /// This returns a reference to the MocoSolver, which you can then edit.
    /// If using this method in C++, make sure to include the "&" in the
    /// return type; otherwise, you'll make a copy of the solver, and the copy
    /// will have no effect on this MocoTool.
    MocoTropterSolver& initSolver();

    /// Access the solver. Make sure to call `initSolver()` beforehand.
    /// If using this method in C++, make sure to include the "&" in the
    /// return type; otherwise, you'll make a copy of the solver, and the copy
    /// will have no effect on this MocoTool.
    MocoTropterSolver& updSolver();

    /// Solve the provided MocoProblem using the provided MocoSolver, and
    /// obtain the solution to the problem. If the write_solution property
    /// contains a file path (that is, it's not "false"), then the solution is
    /// also written to disk.
    /// @precondition
    ///     You must have finished setting up both the problem and solver.
    /// If you have not yet called initSolver(), or if you modified the problem
    /// after calling initSolver(), then solve() will first call initSolver().
    /// This can be called multiple times.
    MocoSolution solve() const;

    /// Interactively visualize an iterate using the simbody-visualizer. The
    /// iterate could be an initial guess, a solution, etc.
    /// @precondition
    ///     The MocoProblem must contain the model corresponding to
    ///     the provided iterate.
    // TODO should visualize be here or in MocoProblem? Should MocoProblem
    // know aobut MocoIterate?
    void visualize(const MocoIterate& it) const;

    /// @name Using solvers other than MocoTropterSolver
    /// In the future, we hope to support custom solvers (but it's not
    /// tested yet).
    /// @{
    template <typename SolverType>
    void setCustomSolver();

    template <typename SolverType>
    SolverType& initCustomSolver() {
        return dynamic_cast<SolverType&>(initSolverInternal());
    }

    template <typename SolverType>
    SolverType& updCustomSolver() {
        return dynamic_cast<SolverType&>(upd_solver());
    }
    /// @}

protected:
    OpenSim_DECLARE_PROPERTY(problem, MocoProblem,
    "The optimal control problem to solve.");
    OpenSim_DECLARE_PROPERTY(solver, MocoSolver,
    "The optimal control algorithm for solving the problem.");

private:

    void ensureInitSolver();
    MocoSolver& initSolverInternal();
    void constructProperties();
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOTOOL_H
