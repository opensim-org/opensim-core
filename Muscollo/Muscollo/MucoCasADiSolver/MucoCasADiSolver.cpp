/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoCasADiSolver.cpp                                     *
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

#include "MucoCasADiSolver.h"
#include "../MuscolloUtilities.h"
#include "CasADiTrapezoidal.h"

#include <casadi/casadi.hpp>

// TODO
// - get all tests to pass using CasADi, adding in features as necessary.
// - create separate tests for tropter and CasADi.
// - copy TropterSolver options to CasADi (num_mesh_points).
// - initial guess
// - converting an iterate between muco and casadi.
// - creating parameters.
// - compute kinematic diff eqs directly if possible (not finite differences).
// - parameters are very inefficient: reapplying parameters more than necessary.
// - how to handle avoiding interpolation of splines? mesh index?
//

using casadi::MX;
using casadi::DM;
using casadi::Sparsity;
using casadi::Slice;
using casadi::Callback;
using casadi::Dict;

using namespace OpenSim;

MucoCasADiSolver::MucoCasADiSolver() {
    constructProperties();
}

MucoIterate MucoCasADiSolver::createGuess(const std::string& type) const {
    OPENSIM_THROW_IF_FRMOBJ(
            type != "bounds"
                    && type != "random"
                    && type != "time-stepping",
            Exception,
            "Unexpected guess type '" + type +
                    "'; supported types are 'bounds', 'random', and "
                    "'time-stepping'.");

    if (type == "time-stepping") {
        return createGuessTimeStepping();
    }

    // TODO avoid performing error checks multiple times; use
    // isObjectUpToDateWithProperties();
    checkPropertyIsPositive(*this, getProperty_num_mesh_points());
    int N = get_num_mesh_points();
    /* TODO

    if (type == "bounds") {
        tropIter = dircol.make_initial_guess_from_bounds();
    } else if (type == "random") {
        tropIter = dircol.make_random_iterate_within_bounds();
    }
    return ocp->convertToMucoIterate(tropIter);
    */
    return MucoIterate();
}

void MucoCasADiSolver::setGuess(MucoIterate guess) {
    // Ensure the guess is compatible with this solver/problem.
    // Make sure to initialize the problem. TODO put in a better place.
    // TODO createTropterProblem();
    guess.isCompatible(getProblemRep(), true);
    clearGuess();
    m_guessFromAPI = std::move(guess);
}
void MucoCasADiSolver::setGuessFile(const std::string& file) {
    clearGuess();
    set_guess_file(file);
}
void MucoCasADiSolver::clearGuess() {
    m_guessFromAPI = MucoIterate();
    m_guessFromFile = MucoIterate();
    set_guess_file("");
    m_guessToUse.reset();
}
const MucoIterate& MucoCasADiSolver::getGuess() const {
    if (!m_guessToUse) {
        if (get_guess_file() != "" && m_guessFromFile.empty()) {
            // The API should make it impossible for both guessFromFile and
            // guessFromAPI to be non-empty.
            assert(m_guessFromAPI.empty());
            // No need to load from file again if we've already loaded it.
            MucoIterate guessFromFile(get_guess_file());
            guessFromFile.isCompatible(getProblemRep(), true);
            m_guessFromFile = guessFromFile;
            m_guessToUse.reset(&m_guessFromFile);
        } else {
            // This will either be a guess specified via the API, or empty to
            // signal that tropter should use the default guess.
            m_guessToUse.reset(&m_guessFromAPI);
        }
    }
    return m_guessToUse.getRef();
}

void MucoCasADiSolver::constructProperties() {
    constructProperty_num_mesh_points(100);
}

MucoSolution MucoCasADiSolver::solveImpl() const {
    checkPropertyIsPositive(*this, getProperty_num_mesh_points());
    CasADiTrapezoidal transcription(*this, getProblemRep());
    transcription.initialize();
    // opt.disp(std::cout, true);
    try {
        return transcription.solve();
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
        // TODO: Return a solution.
    }

    // Some useful functions for debugging:
    // opt.debug().show_infeasibilities();
    // DM controlValues = opt.debug().value(controls);
    // std::cout << "DEBUGg43 " << opt.debug().g_describe(43) << std::endl;
    // std::cout << "DEBUGg43 " << opt.g()(43) << std::endl;
    // std::cout << "DEBUGg43 " << opt.x() << std::endl;

    return {};
}

