#ifndef MUSCOLLO_MUCOTOOL_H
#define MUSCOLLO_MUCOTOOL_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoTool.h                                               *
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

#include "MucoSolver.h"

namespace OpenSim {

class MucoProblem;
class MucoTropterSolver;

/// The top-level class for solving a custom optimal control problem.
///
/// This class consists of a MucoProblem, which describes the optimal control
/// problem, and a MucoSolver, which describes the numerical method for
/// solving the problem.
///
/// Workflow
/// --------
/// When building a MucoTool programmatically (e.g., in C++), the workflow is as
/// follows:
///
/// 1. Build the MucoProblem (set the model, constraints, etc.).
/// 2. Call MucoTool::initSolver(), which returns a reference to the MucoSolver.
///    After this, you cannot edit the MucoProblem.
/// 3. Edit the settings of the MucoSolver (returned by initSolver()).
/// 4. Call MucoTool::solve(). This returns the MucoSolution.
/// 5. (Optional) Postprocess the solution, perhaps using MucoTool::visualize().
///
/// After calling solve(), you can edit the MucoProblem and/or the MucoSolver.
/// You can then call solve() again, if you wish.
///
/// Saving the tool setup to a file
/// -------------------------------
/// You can save the MucoTool to a file by calling MucoTool::print(), and you
/// can load the setup using MucoTool(const std::string& omucoFile).
/// MucoTool setup files have a `.omuco` extension.
///
/// Solver
/// ------
/// The default and only built-in solver uses the **tropter** direct
/// collocation library. We would like to support users plugging in their own
/// solvers, but there is no timeline for this. If you require additional
/// features or enhancements to the solver, please consider contributing to
/// **tropter**.

// TODO rename to MucoFramework.

class OSIMMUSCOLLO_API MucoTool : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoTool, Object);
public:
    OpenSim_DECLARE_PROPERTY(write_solution, std::string,
    "Provide the folder path (relative to working directory) to which the "
    "solution files should be written. Set to 'false' to not write the "
    "solution to disk.");

    MucoTool();

    /// Load a MucoTool setup file.
    MucoTool(const std::string& omucoFile);

    /// Throws an exception if you try calling this after initSolver() and
    /// before solve().
    /// If using this method in C++, make sure to include the "&" in the
    /// return type; otherwise, you'll make a copy of the problem, and the copy
    /// will have no effect on this MucoTool.
    MucoProblem& updProblem();

    /// Call this method once you have finished setting up your MucoProblem.
    /// This returns a reference to the MucoSolver, which you can then edit.
    /// If using this method in C++, make sure to include the "&" in the
    /// return type; otherwise, you'll make a copy of the solver, and the copy
    /// will have no effect on this MucoTool.
    MucoTropterSolver& initSolver();

    /// Access the solver. The solver will be initialized only if it hasn't been
    /// initialized already.
    /// If using this method in C++, make sure to include the "&" in the
    /// return type; otherwise, you'll make a copy of the solver, and the copy
    /// will have no effect on this MucoTool.
    MucoTropterSolver& updSolver();

    /// Solve the provided MucoProblem using the provided MucoSolver, and
    /// obtain the solution to the problem. If the write_solution property
    /// contains a file path (that is, it's not "false"), then the solution is
    /// also written to disk.
    /// @precondition
    ///     You must have finished setting up both the problem and solver.
    /// If you have not yet called initSolver(), or if you modified the problem
    /// after calling initSolver(), then solve() will first call initSolver().
    /// This can be called multiple times.
    MucoSolution solve() const;

    /// Interactively visualize an iterate using the simbody-visualizer. The
    /// iterate could be an initial guess, a solution, etc.
    /// @precondition
    ///     The MucoProblem must contain the model corresponding to
    ///     the provided iterate.
    // TODO should visualize be here or in MucoProblem? Should MucoProblem
    // know aobut MucoIterate?
    void visualize(const MucoIterate& it) const;

    /// @name Using solvers other than MucoTropterSolver
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
        ensureInitSolver();
        return dynamic_cast<SolverType&>(upd_solver());
    }
    /// @}

protected:
    OpenSim_DECLARE_PROPERTY(problem, MucoProblem,
    "The optimal control problem to solve.");
    OpenSim_DECLARE_PROPERTY(solver, MucoSolver,
    "The optimal control algorithm for solving the problem.");

private:

    void ensureInitSolver();
    MucoSolver& initSolverInternal();
    void constructProperties();

    SimTK::ResetOnCopy<bool> m_solverInitialized = false;
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOTOOL_H
