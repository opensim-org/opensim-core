#ifndef MOCO_MOCOSTUDY_H
#define MOCO_MOCOSTUDY_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoStudy.h                                                   *
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

#include "MocoSolver.h"

#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

class MocoProblem;
class MocoTropterSolver;
class MocoCasADiSolver;

/** The top-level class for solving a custom optimal control problem.

This class consists of a MocoProblem, which describes the optimal control
problem, and a MocoSolver, which describes the numerical method for
solving the problem.

Workflow
--------
When building a MocoStudy programmatically (e.g., in C++), the workflow is
as follows:

1. Build the MocoProblem (set the model, constraints, etc.).
2. Call MocoStudy::initSolver(), which returns a reference to the
MocoSolver.
   After this, you cannot edit the MocoProblem.
3. Edit the settings of the MocoSolver (returned by initSolver()).
4. Call MocoStudy::solve(). This returns the MocoSolution.
5. (Optional) Postprocess the solution, perhaps using
MocoStudy::visualize().

After calling solve(), you can edit the MocoProblem and/or the MocoSolver.
You can then call solve() again, if you wish.

Saving the study setup to a file
--------------------------------
You can save the MocoStudy to a file by calling MocoStudy::print(), and you
can load the setup using MocoStudy(const std::string& omocoFile).
MocoStudy setup files have a `.omoco` extension.

Solver
------
The default solver uses the **tropter** direct
collocation library. We also provide the **CasADi** solver, which
depends on the **CasADi** automatic differentiation and optimization
library. If you want to use CasADi programmatically, call initCasADiSolver()
before solve(). We would like to support users plugging in their own
solvers, but there is no timeline for this. If you require additional
features or enhancements to the solver, please consider contributing to
**tropter**. */
class OSIMMOCO_API MocoStudy : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoStudy, Object);

public:
    OpenSim_DECLARE_PROPERTY(write_solution, std::string,
            "Provide the folder path (relative to working directory) to which "
            "the "
            "solution files should be written. Set to 'false' to not write the "
            "solution to disk.");

    MocoStudy();

    /// Load a MocoStudy setup file.
    MocoStudy(const std::string& omocoFile);

    const MocoProblem& getProblem() const;

    /// If using this method in C++, make sure to include the "&" in the
    /// return type; otherwise, you'll make a copy of the problem, and the copy
    /// will have no effect on this MocoStudy.
    MocoProblem& updProblem();

    /// Call this method once you have finished setting up your MocoProblem.
    /// This returns a reference to the MocoSolver, which you can then edit.
    /// If using this method in C++, make sure to include the "&" in the
    /// return type; otherwise, you'll make a copy of the solver, and the copy
    /// will have no effect on this MocoStudy.
    /// This deletes the previous solver if one exists.
    MocoCasADiSolver& initCasADiSolver();

    /// Call this method once you have finished setting up your MocoProblem.
    /// This returns a reference to the MocoSolver, which you can then edit.
    /// If using this method in C++, make sure to include the "&" in the
    /// return type; otherwise, you'll make a copy of the solver, and the copy
    /// will have no effect on this MocoStudy.
    /// This deletes the previous solver if one exists.
    MocoTropterSolver& initTropterSolver();

    /// Access the solver. Make sure to call `initSolver()` beforehand.
    /// If using this method in C++, make sure to include the "&" in the
    /// return type; otherwise, you'll make a copy of the solver, and the copy
    /// will have no effect on this MocoStudy.
    MocoSolver& updSolver();

    /// Solve the provided MocoProblem using the provided MocoSolver, and
    /// obtain the solution to the problem. If the write_solution property
    /// contains a file path (that is, it's not "false"), then the solution is
    /// also written to disk.
    /// @precondition
    ///     You must have finished setting up both the problem and solver.
    /// This reinitializes the solver so that any changes you have made will
    /// hold.
    MocoSolution solve() const;

    /// Interactively visualize a trajectory using the simbody-visualizer. The
    /// trajectory could be an initial guess, a solution, etc.
    /// @precondition
    ///     The MocoProblem must contain the model corresponding to
    ///     the provided trajectory.
    void visualize(const MocoTrajectory& it) const;

    /// Calculate the requested outputs using the model in the problem and the
    /// states and controls in the MocoTrajectory.
    /// The output paths can be regular expressions. For example,
    /// ".*activation" gives the activation of all muscles.
    /// Constraints are not enforced but prescribed motion (e.g.,
    /// PositionMotion) is.
    /// @see OpenSim::analyze()
    /// @note Parameters in the MocoTrajectory are **not** applied to the model.
    TimeSeriesTable analyze(
            const MocoTrajectory& it, std::vector<std::string> outputPaths) const;

    /// @name Using other solvers
    /// @{
    template <typename SolverType>
    void setCustomSolver() {
        set_solver(SolverType());
    }

    /// @precondition If not using MocoTropterSolver or MocoCasADiSolver, you
    /// must invoke setCustomSolver() first.
    template <typename SolverType>
    SolverType& initSolver() {
        return dynamic_cast<SolverType&>(initSolverInternal());
    }

    template <typename SolverType>
    SolverType& updSolver() {
        return dynamic_cast<SolverType&>(upd_solver());
    }
    /// @}

protected:
    OpenSim_DECLARE_PROPERTY(
            problem, MocoProblem, "The optimal control problem to solve.");
    OpenSim_DECLARE_PROPERTY(solver, MocoSolver,
            "The optimal control algorithm for solving the problem.");

private:
    MocoSolver& initSolverInternal();
    void constructProperties();
};

template <>
OSIMMOCO_API MocoTropterSolver& MocoStudy::initSolver<MocoTropterSolver>();

template <>
OSIMMOCO_API MocoCasADiSolver& MocoStudy::initSolver<MocoCasADiSolver>();

} // namespace OpenSim

#endif // MOCO_MOCOSTUDY_H
