/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoSolver.cpp                                           *
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
#include "MucoSolver.h"

#include "MucoProblem.h"

#include <OpenSim/Simulation/Manager/Manager.h>

using namespace OpenSim;

MucoIterate MucoSolver::createGuessTimeStepping() const {
    const auto& probrep = getProblemRep();
    const auto& initialTime = probrep.getTimeInitialBounds().getUpper();
    const auto& finalTime = probrep.getTimeFinalBounds().getLower();
    OPENSIM_THROW_IF_FRMOBJ(finalTime <= initialTime, Exception,
            "Expected lower bound on final time to be greater than "
            "upper bound on initial time, but "
            "final_time.lower: " + std::to_string(finalTime) + "; " +
                    "initial_time.upper: " + std::to_string(initialTime) + ".");
    Model model(probrep.getModel());

    // Disable all controllers?
    SimTK::State state = model.initSystem();

    // Modify initial state values as necessary.
    Array<std::string> svNames = model.getStateVariableNames();
    for (int isv = 0; isv < svNames.getSize(); ++isv) {
        const auto& svName = svNames[isv];
        const auto& initBounds =
                probrep.getStateInfo(svName).getInitialBounds();
        const auto defaultValue = model.getStateVariableValue(state, svName);
        SimTK::Real valueToUse = defaultValue;
        if (initBounds.isEquality()) {
            valueToUse = initBounds.getLower();
        } else if (!initBounds.isWithinBounds(defaultValue)) {
            valueToUse = 0.5 * (initBounds.getLower() + initBounds.getUpper());
        }
        if (valueToUse != defaultValue) {
            model.setStateVariableValue(state, svName, valueToUse);
        }
    }

    // TODO Equilibrate fiber length?

    state.setTime(initialTime);
    Manager manager(model, state);
    manager.integrate(finalTime);

    const auto& statesTable = manager.getStatesTable();
    auto controlsTable = model.getControlsTable();

    // Fix column labels.
    auto labels = controlsTable.getColumnLabels();
    for (auto& label : labels) { label = "/forceset/" + label; }
    controlsTable.setColumnLabels(labels);

    // TODO handle parameters.
    return MucoIterate::createFromStatesControlsTables(
            probrep, statesTable, controlsTable);
}

void MucoSolver::resetProblem(const MucoProblem& problem) {
    m_problem.reset(&problem);
    m_problemRep = problem.createRep();
    resetProblemImpl(m_problemRep);
}

MucoSolution MucoSolver::solve() const {
    OPENSIM_THROW_IF(!m_problem, Exception, "Problem not set.");
    return solveImpl();
}

void MucoSolver::setSolutionStats(MucoSolution& sol,
        bool success, const std::string& status, int numIterations) {
    sol.setSuccess(success);
    sol.setStatus(status);
    sol.setNumIterations(numIterations);
}


