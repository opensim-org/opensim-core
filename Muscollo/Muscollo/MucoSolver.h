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

#include "MucoProblemRep.h"

#include <OpenSim/Common/Object.h>

#include <SimTKcommon/internal/ReferencePtr.h>

namespace OpenSim {

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

    MucoSolver() = default;

    /// This calls resetProblem() with the provided problem.
    explicit MucoSolver(const MucoProblem& problem);

    virtual ~MucoSolver() = default;

    /// Call this to prepare the solver for use on the provided problem.
    // TODO can only call once?
    // TODO @precondition The problem is well-posed (MucoProblem::isWellPosed
    // ()). Move isWellPosed() to Solver, since evaluating this might require
    // creating the solver.
    void resetProblem(const MucoProblem& problem);

protected:

    //OpenSim_DECLARE_LIST_PROPERTY(options, MucoSolverOption, "TODO");

    /// This is a service for derived classes, because
    /// MucoSolution::setStatus(), MucoSolution::setSuccess(), etc. are private
    /// but this class is a friend of MucoSolution.
    static void setSolutionStats(MucoSolution&,
            bool success, const std::string& status, int numIterations);

    const MucoProblemRep& getProblemRep() const {
        return m_problemRep;
    }

private:

    /// This is called by MucoTool.
    // We don't want to make this public, as users would get confused about
    // whether they should call MucoTool::solve() or MucoSolver::solve().
    MucoSolution solve() const;
    friend MucoTool;

    /// Check that solver is capable of solving this problem.
    virtual void resetProblemImpl(const MucoProblemRep& rep) const = 0;

    /// This is the meat of a solver: solve the problem and return the solution.
    virtual MucoSolution solveImpl() const = 0;

    SimTK::ReferencePtr<const MucoProblem> m_problem;
    mutable SimTK::ResetOnCopy<MucoProblemRep> m_problemRep;

};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOSOLVER_H
