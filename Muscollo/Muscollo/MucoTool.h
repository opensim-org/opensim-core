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

#include "MucoSolution.h"

namespace OpenSim {

class MucoProblem;
class MucoSolver;

/// Saving the tool setup to a file
/// ===============================
/// You can save the MucoTool to a file by calling MucoTool::print().
/// MucoTool setup files have a `.omuco` extension.

// TODO work flow.
class MucoTool : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoTool, Object);
public:
    MucoTool();

    /// Throws an exception if you try calling this after initSolver() and
    /// before solve().
    MucoProblem& updProblem();
    /// If you want to tweak settings on the solver
    /// After calling this, you cannot
    MucoSolver& initSolver();
    // TODO add a "reset()"
    MucoSolution solve();

private:
    OpenSim_DECLARE_PROPERTY(model, Model, "TODO");

    SimTK::ClonePtr<MucoProblem> _problem;
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOTOOL_H
