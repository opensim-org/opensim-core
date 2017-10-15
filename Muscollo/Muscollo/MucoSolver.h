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

class OSIMMUSCOLLO_API MucoSolver : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(MucoSolver, Object);
public:

    MucoSolver();

    /// Initialize the solver with the provided problem.
    explicit MucoSolver(const MucoProblem& problem);

    void resetProblem();

    // TODO can only call once?
    void resetProblem(const MucoProblem& problem);

    const MucoProblem& getProblem() const {
        OPENSIM_THROW_IF(!m_problem, Exception, "Problem not set; call "
                "resetProblem() with a problem.");
        return m_problem.getRef();
    }

    // You can call this multiple times.
    MucoSolution solve() const;

private:

    //OpenSim_DECLARE_LIST_PROPERTY(options, MucoSolverOption, "TODO");

    virtual void resetProblemImpl() = 0;
    virtual void resetProblemImpl(const MucoProblem& problem) = 0;
    virtual MucoSolution solveImpl() const = 0;

    SimTK::ReferencePtr<const MucoProblem> m_problem;

};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOSOLVER_H
