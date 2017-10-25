#ifndef MUSCOLLO_MUCOSOLVER_H
#define MUSCOLLO_MUCOSOLVER_H

/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoSolver.h                                             *
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

#include "MucoIterate.h"

#include <OpenSim/Common/Object.h>

#include <SimTKcommon/internal/ReferencePtr.h>

namespace OpenSim {

class MucoProblem;
class MucoTool;

// TODO create typed versions?
/*
class MucoSolverOption : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoSolverOption, Object);
public:
    OpenSim_DECLARE_PROPERTY(value, std::string, "TODO");
    MucoSolverOption() {
        constructProperties();
    }
};
 */

// TODO what's the desired behavior upon copy?

/// Once the solver is created, you should not make any edits to the
/// MucoProblem. If you do, you must call resetProblem(const MucoProblem&
/// problem).
class OSIMMUSCOLLO_API MucoSolver : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(MucoSolver, Object);
public:

    MucoSolver();

    /// This calls resetProblem() with the provided problem.
    explicit MucoSolver(const MucoProblem& problem);

    /// Clear the internally-set MucoProblem.
    void resetProblem();

    /// Call this to prepare the solver for use on the provided problem.
    // TODO can only call once?
    // TODO @precondition The problem is well-posed (MucoProblem::isWellPosed
    // ()). Move isWellPosed() to Solver, since evaluating this might require
    // creating the solver.
    // TODO use a wrapper class (like MODelegate).
    void resetProblem(const MucoProblem& problem);

    const MucoProblem& getProblem() const {
        OPENSIM_THROW_IF(!m_problem, Exception, "Problem not set; call "
                "resetProblem() with a problem.");
        return m_problem.getRef();
    }

protected:

    //OpenSim_DECLARE_LIST_PROPERTY(options, MucoSolverOption, "TODO");

private:

    /// This is called by MucoTool.
    // We don't want to make this public, as users would get confused about
    // whether they should call MucoTool::solve() or MucoSolver::solve().
    MucoSolution solve() const;
    friend MucoTool;

    /// Claer any cache based on the MucoProblem.
    virtual void resetProblemImpl() = 0;
    /// Perform any necessary caching based on the MucoProblem.
    /// @
    virtual void resetProblemImpl(const MucoProblem& problem) = 0;

    /// This is the meat of a solver: solve the problem and return the solution.
    virtual MucoSolution solveImpl() const = 0;

    SimTK::ReferencePtr<const MucoProblem> m_problem;

};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOSOLVER_H
