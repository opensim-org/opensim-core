/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoTool.cpp                                             *
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
#include "MucoTool.h"
#include "MucoProblem.h"
#include "MucoTropterSolver.h"

using namespace OpenSim;

MucoTool::MucoTool() {
    constructProperties();
}

void MucoTool::constructProperties() {
    constructProperty_problem(MucoProblem());
    constructProperty_solver(MucoTropterSolver());
}

MucoProblem& MucoTool::updProblem() {
    upd_solver().resetProblem();
    return upd_problem();
}

MucoTropterSolver& MucoTool::initSolver() {
    return initSolver<MucoTropterSolver>();
}

MucoSolution MucoTool::solve() {
    return get_solver().solve();
}
