/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoCasOCSolver.cpp                                          *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2018 Stanford University and the Authors                     *
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

#include "CasOCProblem.h"

#include "../MocoUtilities.h"
#include "CasOCTranscription.h"
#include "CasOCTrapezoidal.h"

using OpenSim::Exception;
using OpenSim::format;

namespace CasOC {

std::unique_ptr<Transcription> Solver::createTranscription() const {
    std::unique_ptr<Transcription> transcription;
    if (m_transcriptionScheme == "trapezoidal") {
        transcription = OpenSim::make_unique<Trapezoidal>(*this, m_problem);
    } else {
        OPENSIM_THROW(Exception, format("Unknown transcription scheme '%s'.",
                                         m_transcriptionScheme));
    }
    return transcription;
}

Iterate Solver::createInitialGuessFromBounds() const {
    auto transcription = createTranscription();
    return transcription->createInitialGuessFromBounds();
}

Iterate Solver::createRandomIterateWithinBounds() const {
    auto transcription = createTranscription();
    return transcription->createRandomIterateWithinBounds();
}

Solution Solver::solve(const Iterate& guess) const {
    auto transcription = createTranscription();
    return transcription->solve(guess);
}

} // namespace CasOC
