/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoTool.cpp                                                 *
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
#include "MocoTool.h"

#include "Components/PositionMotion.h"
#include "MocoCasADiSolver/MocoCasADiSolver.h"
#include "MocoProblem.h"
#include "MocoTropterSolver.h"
#include "MocoUtilities.h"
#include <regex>

#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Simulation/StatesTrajectory.h>

using namespace OpenSim;

MocoTool::MocoTool() { constructProperties(); }

MocoTool::MocoTool(const std::string& omocoFile) : Object(omocoFile) {
    constructProperties();
    updateFromXMLDocument();
}

void MocoTool::constructProperties() {
    constructProperty_write_solution("./");
    constructProperty_problem(MocoProblem());
    constructProperty_solver(MocoTropterSolver());
}

const MocoProblem& MocoTool::getProblem() const { return get_problem(); }

MocoProblem& MocoTool::updProblem() { return upd_problem(); }

MocoSolver& MocoTool::initSolverInternal() {
    // TODO what to do if we already have a solver (from cloning?)
    upd_solver().resetProblem(get_problem());
    return upd_solver();
}

template <>
MocoTropterSolver& MocoTool::initSolver<MocoTropterSolver>() {
    set_solver(MocoTropterSolver());
    return dynamic_cast<MocoTropterSolver&>(initSolverInternal());
}

template <>
MocoCasADiSolver& MocoTool::initSolver<MocoCasADiSolver>() {
    set_solver(MocoCasADiSolver());
    return dynamic_cast<MocoCasADiSolver&>(initSolverInternal());
}

MocoTropterSolver& MocoTool::initTropterSolver() {
    set_solver(MocoTropterSolver());
    return initSolver<MocoTropterSolver>();
}

MocoCasADiSolver& MocoTool::initCasADiSolver() {
    return initSolver<MocoCasADiSolver>();
}

MocoSolver& MocoTool::updSolver() { return updSolver<MocoSolver>(); }

MocoSolution MocoTool::solve() const {
    // TODO avoid const_cast.
    const_cast<Self*>(this)->initSolverInternal();
    MocoSolution solution = get_solver().solve();
    bool originallySealed = solution.isSealed();
    if (get_write_solution() != "false") {
        OpenSim::IO::makeDir(get_write_solution());
        std::string prefix = getName().empty() ? "MocoTool" : getName();
        solution.unseal();
        const std::string filename = get_write_solution() +
                                     SimTK::Pathname::getPathSeparator() +
                                     prefix + "_solution.sto";
        try {
            solution.write(filename);
        } catch (const TimestampGreaterThanEqualToNext&) {
            std::cout << "Could not write solution to file...skipping."
                      << std::endl;
        }
        if (originallySealed) solution.seal();
    }
    return solution;
}

void MocoTool::visualize(const MocoIterate& it) const {
    // TODO this does not need the Solver at all, so this could be moved to
    // MocoProblem.
    const auto& model = get_problem().getPhase(0).getModel();
    OpenSim::visualize(model, it.exportToStatesStorage());
}

TimeSeriesTable MocoTool::analyze(const MocoIterate& iterate,
        std::vector<std::string> outputPaths) const {
    auto model = get_problem().createRep().getModelBase();

    prescribeControlsToModel(iterate, model);

    auto* reporter = new TableReporter();
    for (const auto& comp : model.getComponentList()) {
        for (const auto& outputName : comp.getOutputNames()) {
            const auto& output = comp.getOutput(outputName);
            if (output.getTypeName() == "double") {
                const auto& thisOutputPath = output.getPathName();
                for (const auto& outputPathArg : outputPaths) {
                    if (std::regex_match(
                                thisOutputPath, std::regex(outputPathArg))) {
                        reporter->addToReport(output);
                    }
                }
            }
        }
    }
    model.addComponent(reporter);

    model.initSystem();

    auto statesTraj = iterate.exportToStatesTrajectory(get_problem());

    for (auto state : statesTraj) {
        model.getSystem().prescribe(state);
        model.realizeReport(state);
    }

    return reporter->getTable();
}
