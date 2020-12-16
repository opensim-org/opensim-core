#ifndef OPENSIM_MOCOSOLVER_H
#define OPENSIM_MOCOSOLVER_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoSolver.h                                                      *
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

#include "MocoProblemRep.h"
#include "MocoTrajectory.h"

#include <SimTKcommon/internal/ReferencePtr.h>

#include <OpenSim/Common/Object.h>

namespace OpenSim {

class MocoStudy;

/** Once the solver is created, you should not make any edits to the
MocoProblem. If you do, you must call resetProblem(const MocoProblem&
problem). */
class OSIMMOCO_API MocoSolver : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(MocoSolver, Object);
public:

    MocoSolver() = default;

    /// This calls resetProblem() with the provided problem.
    explicit MocoSolver(const MocoProblem& problem);

    virtual ~MocoSolver() = default;

    /// Call this to prepare the solver for use on the provided problem.
    /// The solver creates and stores a MocoProblemRep using the provided
    /// problem.
    // This function is const because we do not consider the reference to the
    // problem to be logically part of the solver.
    void resetProblem(const MocoProblem& problem) const;

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
    /// The number of times in the trajectory is the number of successful
    /// integration steps.
    ///
    /// @note This function does not yet support problems with parameters.
    ///
    /// @note This function assumes all actuators are in the model's ForceSet.
    ///
    /// @throws Exception If the lower bound on the final time is less than or
    /// equal to the upper bound on the initial time. This situation is okay in
    /// general; it's just that this function doesn't support it.
    ///
    /// @precondition You must have called resetProblem().
    MocoTrajectory createGuessTimeStepping() const;

protected:

    //OpenSim_DECLARE_LIST_PROPERTY(options, MocoSolverOption, "TODO");

    /// This is a service for derived classes, because
    /// MocoSolution::setStatus(), MocoSolution::setSuccess(), etc. are private
    /// but this class is a friend of MocoSolution.
    static void setSolutionStats(MocoSolution&,
            bool success, double objective,
            const std::string& status, int numIterations,
            double duration,
            std::vector<std::pair<std::string, double>> objectiveBreakdown =
                    {});

    const MocoProblemRep& getProblemRep() const {
        return m_problemRep;
    }

    /// Create a library of MocoProblemRep%s for use in parallelized code.
    // TODO SWIG ignore.
    std::unique_ptr<ThreadsafeJar<const MocoProblemRep>>
    createProblemRepJar(int size) const;

private:

    /// This is called by MocoStudy.
    // We don't want to make this public, as users would get confused about
    // whether they should call MocoStudy::solve() or MocoSolver::solve().
    MocoSolution solve() const;
    friend MocoStudy;

    /// This is the meat of a solver: solve the problem and return the solution.
    virtual MocoSolution solveImpl() const = 0;

    mutable SimTK::ReferencePtr<const MocoProblem> m_problem;
    mutable SimTK::ResetOnCopy<MocoProblemRep> m_problemRep;

};

} // namespace OpenSim

#endif // OPENSIM_MOCOSOLVER_H
